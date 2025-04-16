#include "core/device/vdb/registry.hpp"

namespace VDP {
/**
 * checks a packets validility
 * @param packet the packet to check the validity of
 * @return the PacketValidility (TooSmall, BadChecksum, or Ok)
 */
static VDP::PacketValidity validate_packet(const VDP::Packet &packet) {
    VDPTracef("Validating packet of size %d", (int)packet.size());

    // packet header byte + channel byte + checksum = 6 bytes
    static constexpr size_t min_packet_size = 6;

    // checks that the minimum packet size is met
    if (packet.size() < min_packet_size) {
        return VDP::PacketValidity::TooSmall;
    }
    // calculates the checksum for the packet
    uint32_t checksum = CRC32::calculate(packet.data(), packet.size() - 4);

    // recreates the checksum manually
    auto size = packet.size();
    const uint32_t written_checksum = (uint32_t(packet[size - 1]) << 24) | (uint32_t(packet[size - 2]) << 16) |
                                      (uint32_t(packet[size - 3]) << 8) | uint32_t(packet[size - 4]);
    // checks if both checksums match
    if (checksum != written_checksum) {
        VDPWarnf("Checksums do not match: expected: %08lx, got: %08lx", checksum, written_checksum);
        return VDP::PacketValidity::BadChecksum;
    }
    // if no problems with the packet are found, packet is Ok
    return VDP::PacketValidity::Ok;
}
/**
 * creates a device registry for listening to data over the device
 * @param device the device to send data to
 * @param reg_type the type of registry it is (Listener or Controller)
 */
RegistryListener::RegistryListener(AbstractDevice *device) : device(device) {
    device->register_receive_callback([&](const Packet &p) {
        printf("GOT PACKET\n");
        take_packet(p);
    });
}
/**
 * creates a device registry for sending data over the device
 * @param device the device to send data to
 * @param reg_type the type of registry it is (Listener or Controller)
 */
RegistryController::RegistryController(AbstractDevice *device) : device(device) {
    device->register_receive_callback([&](const Packet &p) {
        printf("SENT PACKET\n");
        take_packet(p);
    });
}
/**
 * Handles various types of packets, validating them and passing them through the system
 * according to its type and function
 */
void RegistryListener::take_packet(const Packet &pac) {
    VDPTracef("Received packet of size %d", (int)pac.size());
    // checks the validity of the packet
    const VDP::PacketValidity status = validate_packet(pac);

    if (status == VDP::PacketValidity::BadChecksum) {
        VDPWarnf("Listener: Bad packet checksum. Skipping", "");
        num_bad++;
        return;
    } else if (status == VDP::PacketValidity::TooSmall) {
        num_small++;
        VDPWarnf("Listener: Packet too small to be valid (%d bytes). Skipping", (int)pac.size());
        dump_packet(pac);
        return;
    } else if (status != VDP::PacketValidity::Ok) {
        VDPWarnf("Listener: Unknown validity of packet (BAD). Skipping", "");
        return;
    }
    // checks the packet function from the header
    const VDP::PacketHeader header = VDP::decode_header_byte(pac[0]);
    if (header.func == VDP::PacketFunction::Send) {
        VDPTracef("Listener: PacketFunction Send");

        if (header.type == VDP::PacketType::Data) {
            // if the packet is a data, get the data from the packet
            VDPTracef("Listener: PacketType Data");
            // get the channel id from the second byte of the packet
            const ChannelID id = pac[1];
            // stores the channel id's schema in a Part Pointer
            const PartPtr part = get_remote_schema(id);
            if (part == nullptr) {
                VDPDebugf("VDB-Listener: No channel information for id: %d", id);
                return;
            }
            // creates a PacketReader starting after the channel id location
            PacketReader reader{pac, 2};
            // stores the data read from the packet to the Registry Part
            part->read_data_from_message(reader);
            // runs the channel's on data callback
            on_data(Channel{part, id});
        }
    } else if (header.func == VDP::PacketFunction::Request) {
        // if the packet is a data, get the data from the packet
        VDPTracef("Listener: PacketType Request");
        // creates a PacketReader starting after the channel id location
        Packet scratch;
        PacketWriter writer{scratch};
        writer.write_receive(channel_receive_queue);
        device->send_packet(writer.get_packet());
    }
}

/**
 * Handles a packet obtained from the Listener
 * according to its type and function
 */
void RegistryController::take_packet(const Packet &pac) {
    VDPTracef("Received packet of size %d", (int)pac.size());
    // checks the validity of the packet
    const VDP::PacketValidity status = validate_packet(pac);

    if (status == VDP::PacketValidity::BadChecksum) {
        VDPWarnf("Controller: Bad packet checksum. Skipping", "");
        num_bad++;
        return;
    } else if (status == VDP::PacketValidity::TooSmall) {
        num_small++;
        VDPWarnf("Controller: Packet too small to be valid (%d bytes). Skipping", (int)pac.size());
        dump_packet(pac);
        return;
    } else if (status != VDP::PacketValidity::Ok) {
        VDPWarnf("Controller: Unknown validity of packet (BAD). Skipping", "");
        return;
    }
    // checks the packet function from the header
    const VDP::PacketHeader header = VDP::decode_header_byte(pac[0]);
    if (header.func == VDP::PacketFunction::Acknowledge) {
        // if the packet is an acknowledgement packet
        PacketReader reader(pac, 1);
        /**
         * read the channel id and then set its acknowledgement boolean to true
         */
        const ChannelID id = reader.get_number<ChannelID>();
        if (id >= my_channels.size()) {
            printf("VDB-Controller: Recieved ack for unknown channel %d", id);
        }
        my_channels[id].acked = true;
    } else if (header.func == VDP::PacketFunction::Receive) {
        // if the packet is a data, get the data from the packet
        VDPTracef("Controller: PacketType Data");
        // get the channel id from the second byte of the packet
        const ChannelID id = pac[1];
        // stores the channel id's schema in a Part Pointer
        const PartPtr part = my_channels[id + 1].data;
        if (part == nullptr) {
            VDPDebugf("VDB-Listener: No channel information for id: %d", id);
            return;
        }
        // creates a PacketReader starting after the channel id location
        PacketReader reader{pac, 2};
        // stores the data read from the packet to the Registry Part
        part->read_data_from_message(reader);
        // runs the channel's on data callback
        on_data(Channel{part, id});
    }
}

/**
 * @param id the remote channel id to get data from
 * @param return the Part Pointer of schema data in the channel
 */
PartPtr RegistryListener::get_remote_schema(ChannelID id) {
    if (id >= remote_channels.size()) {
        return nullptr;
    }
    return remote_channels[id].data;
}
/**
 * @param id the remote channel id to get data from
 * @param return the Part Pointer of schema data in the channel
 */
PartPtr RegistryController::get_remote_schema(ChannelID id) {
    if (id >= remote_channels.size()) {
        return nullptr;
    }
    return remote_channels[id].data;
}
/**
 * installs a callback to a function that is called when the registry broadcasts the data schematic
 * @param on_broadcastf the callback to run when the registry broadcasts the schematic
 */
void RegistryListener::install_broadcast_callback(CallbackFn on_broadcastf) {
    VDPTracef("Listener: Installed broadcast callback for ");
    this->on_broadcast = std::move(on_broadcastf);
}
/**
 * installs a callback to a function that is called when the registry broadbasts data
 * @param on_dataf the callback to run when the registry broadcasts data
 */
void RegistryListener::install_data_callback(CallbackFn on_dataf) {
    VDPTracef("Listener: Installed data callback for ");
    this->on_data = std::move(on_dataf);
}
/**
 * creates a new channel for the given VDP object, broadcasts it to the device
 * on the other end of the line. This channel is open to be written to
 * immediately, however it is not guaranteed to be sent to the other side
 * until broadcasting has been completed
 * Channel is only valid once negotiate has been callsed
 * @param for_data the Part Pointer to the data the channel should hold
 * @return the channel id for the new channel created
 */
ChannelID RegistryController::open_channel(PartPtr for_data) {
    ChannelID id = new_channel_id();
    Channel chan = Channel{for_data, id};
    my_channels.push_back(chan);
    return chan.id;
}
/**
 * sets the data at the channel id to a Part Pointer and sends it to the device
 * @param id The id of the channel to hold the data
 * @param data the Part Pointer for the channel to hold and send to the device
 */
bool RegistryListener::send_data(ChannelID id, PartPtr data) {
    // checks if the channel is actually stored in the Registry
    if (id > my_channels.size()) {
        printf("VDB-Listener: Channel with ID %d doesn't exist yet\n", (int)id);
        return false;
    }
    // sets the channel's data to the Part Pointer given
    Channel &chan = my_channels[id];
    chan.data = data;
    // checks if the channel has been acknowledged yet
    if (!chan.acked) {
        printf("VDB-Listener: Channel %d has not yet been negotiated. Dropping packet\n", (int)id);
        return false;
    }
    // if it has been acknowledged write the channel's data to a packet and send it to the device
    VDP::Packet scratch;
    PacketWriter writ{scratch};

    writ.write_data_message(chan);
    VDP::Packet pac = writ.get_packet();

    return device->send_packet(pac);
}
/**
 * sets the data at the channel id to a Part Pointer and sends it to the device
 * @param id The id of the channel to hold the data
 * @param data the Part Pointer for the channel to hold and send to the device
 */
bool RegistryController::send_data(ChannelID id, PartPtr data) {
    // checks if the channel is actually stored in the Registry
    if (id > my_channels.size()) {
        printf("VDB-Controller: Channel with ID %d doesn't exist yet\n", (int)id);
        return false;
    }
    // sets the channel's data to the Part Pointer given
    Channel &chan = my_channels[id];
    chan.data = data;
    // checks if the channel has been acknowledged yet
    if (!chan.acked) {
        printf("VDB-Controller: Channel %d has not yet been negotiated. Dropping packet\n", (int)id);
        return false;
    }
    // if it has been acknowledged write the channel's data to a packet and send it to the device
    VDP::Packet scratch;
    PacketWriter writ{scratch};

    writ.write_data_message(chan);
    VDP::Packet pac = writ.get_packet();

    return device->send_packet(pac);
}

/**
 * sends channel schematics to the Registry device and checks for ackowledgements
 * @return whether or not all channel's were acknowledgements
 */
bool RegistryController::negotiate() {
    printf("Negotiating\n");
    bool acked_all = true;
    int failed_acks = 0;

    constexpr size_t BROADCAST_TRIES_PER = 3;
    // negotiates with each channel however many times broadcast_tries_per is set to
    for (size_t i = 0; i < my_channels.size(); i++) {
        for (size_t j = 0; j < BROADCAST_TRIES_PER; j++) {
            VDPDebugf("Controller: Negotiating chan id %d", i);
            // writes the channel's data schematic to a packet
            const Channel &chan = my_channels[i];
            Packet scratch;
            PacketWriter writer{scratch};
            writer.write_channel_broadcast(chan);
            VDP::Packet pac = writer.get_packet();

            // sends the data schematic to the device
            device->send_packet(pac);
            needs_ack = true;
            // waits 500ms for an acknowledgment of the packet
            auto time = VDB::time_ms();
            while (!chan.acked) {
                auto now = VDB::time_ms();
                if (now - time > ack_ms) {
                    // timed out
                    break;
                }
                VDB::delay_ms(5);
            }
            // checks if the packet was acknowledged
            if (chan.acked == true) {
                // if the channel was acknowledged, move on to the next channel
                VDPTracef(
                  "Controller: Acked channel %d after %d ms on attempt %d", chan.id, (int)(VDB::time_ms() - time),
                  (int)j + 1
                );
                break;
            } else {
                // if the channel was not acknowledged add one to the failed acknowledgements counter and set acked_all
                // to false then move on to the next channel
                VDPWarnf("Controller: ack for chan id:%02x expired after %d msec", chan.id, (int)ack_ms);
                failed_acks++;
                if (j == BROADCAST_TRIES_PER - 1) {
                    acked_all = false;
                }
            }
        }
    }
    // print out how many times a channel failed to be acknowledged
    if (failed_acks > 0) {
        VDPWarnf("Controller: Failed to ack %d times", failed_acks);
    }
    return acked_all;

    // if (needs_ack && waiting_on_ack_timer.time(vex::timeUnits::msec) < ack_ms)
    // { printf("Still waiting on ack"); vexDelay(ack_ms -
    // waiting_on_ack_timer.time(vex::timeUnits::msec));
    // }

    // if (reg_type == Side::Controller) {
    // PacketWriter writer;
    // writer.write_channel_broadcast(chan);
    // VDP::Packet pac = writer.get_packet();
    //
    // device->send_packet(pac);
    // needs_ack = true;
    // waiting_on_ack_timer.reset();
    // }
}

} // namespace VDP
