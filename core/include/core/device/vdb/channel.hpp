#pragma once
#include <cstdio>
#include "core/device/vdb/essential.hpp"

namespace VDP{

using Packet = std::vector<uint8_t>;
constexpr size_t MAX_CHANNELS = 256;

// defines a channel id as an 8bit unsigned integer
using ChannelID = uint8_t;
class Channel {
  public:
    template <typename MutexType> friend class RegistryListener;
    friend class RegistryController;
    /**
     * Creates a channel used for sending data to the brain
     * @param data Part Pointer for the data stored at the channel
     */
    explicit Channel(PartPtr &data) : data(data) {}
    PartPtr data;
    /*
     * @return The Channel ID from 0 - 256
     */
    ChannelID getID() const {return id;}

  private:
    /**
     * Creates a channel used for sending data to the brain
     * @param data Part Pointer for the data stored at the channel
     * @param channel_id The Channel ID to assign the channel from 0 - 256
     */
    Channel(VDP::PartPtr &data, ChannelID channel_id) : data(data), id(channel_id) {}

    ChannelID id = 0;
    VDP::Packet packet_scratch_space;
    bool acked = false;
    // std::vector
};

}