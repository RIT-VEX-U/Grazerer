#pragma once
#include "vdb/protocol.hpp"
#include "vdb_cobs.hpp"
#include "vex.h"

namespace VDB {
class Device : public VDP::AbstractDevice, VDBCOBSSerialDevice {
  public:
    explicit Device(int32_t port, int32_t baud_rate);
    bool send_packet(const VDP::Packet &packet) override; // From VDP::AbstractDevice

    void register_receive_callback(std::function<void(const VDP::Packet &packet)> callback
    ) override; // From VDP::AbstractDevice

    void cobs_packet_callback(const Packet &pac) override;

  private:
    std::function<void(const VDP::Packet &packet)> callback;
};

} // namespace VDB