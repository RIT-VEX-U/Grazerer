#pragma once
#include "core/device/vdb/packet.hpp"
#include <functional>

namespace VDP{

/**
 * defines a generic device to trasmit packets through
 */
class AbstractDevice {
  public:
    /** Sends a packet over some transmission medium
     * It is not specified how the packet reaches the partner
     * The transmission medium and wire format are left to the user
     * @param packet the packet to send through the device
     * @return whether the packet was sent sucessfully or not
     */
    virtual bool send_packet(const VDP::Packet &packet) = 0;
    /**
     * a callback to function that runs when a new packet is available
     * @param the function for the callback to call
     * me when my ex-wife
     */
    virtual void register_receive_callback(std::function<void(const VDP::Packet &packet)> callback) = 0;
    /**
     *  deleter for the device, used to delete it when it is no longer needed
     */
    virtual ~AbstractDevice();
};

}