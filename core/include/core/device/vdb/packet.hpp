#pragma once
#include "core/device/vdb/essential.hpp"

#include <vector>
#include <deque>
namespace VDP{

///////////////////////////////// Packet + Functions /////////////////////////////////

// Packet of bytes stored in a vector of 8 bit unsigned integers
using Packet = std::vector<uint8_t>;

/**
 * defines what byte value is what type in a packet
 */
enum class Type : uint8_t {
    Record = 0,
    String = 1,
    // Enum

    Double = 3,
    Float = 4,

    Uint8 = 5,
    Uint16 = 6,
    Uint32 = 7,
    Uint64 = 8,

    Int8 = 9,
    Int16 = 10,
    Int32 = 11,
    Int64 = 12,

};


/**
 * defines what byte value correspondes to what packet type or packet function
 */
enum class PacketType : uint8_t {
  Broadcast = 0b00000000,
  Data = 0b10000000
};
enum class PacketFunction : uint8_t {
  Send = 0b00000000,
  Acknowledge = 0b00100000,
  Response = 0b01000000,
  Request = 0b01100000
};

/**
 * struct to define the header of a packet,
 * defines wheether a packet is Broadcoast or data
 * and whether a packet is send or recieve
 */
struct PacketHeader {
    PacketType type;
    PacketFunction func;
};
enum PacketValidity : uint8_t {
    Ok,
    BadChecksum,
    TooSmall,
};

/**
 * Prints out a packet of data
 */
void dump_packet_hex(const Packet &pac);
void dump_packet_8bit(const Packet &pac);

PacketValidity validate_packet(const VDP::Packet &packet);

/**
 * creates a byte from a given packet header
 * @return the header byte created
 */
uint8_t make_header_byte(PacketHeader head);

/**
 * @param hb the header byte to decode
 * @return a PacketHeader with the Function and Type from the byte decoded
 */
PacketHeader decode_header_byte(uint8_t hb);

/**
 * Decodes the broadcast in a packet
 * @param packet the packet to decode
 * @return the pair of the Channel ID and the Part Pointer of the packet schematic
 */
std::pair<ChannelID, PartPtr> decode_broadcast(const Packet &packet);

std::pair<ChannelID, PartPtr> decode_data(const Packet &packet);

///////////////////////////////// Packet Reader /////////////////////////////////
class PacketReader {
  public:
    /**
     * Defines a PacketReader to read a packet
     * @param pac the packet to read
     */
    PacketReader(Packet pac);
    /**
     * Defines a PacketReader to read a packet with a set start location for the packet
     * @param pac the packet to read
     * @param start the start location for the reader to start reading from
     */
    PacketReader(Packet pac, size_t start);

    /**
     * creates a decoder to decode a packet
     * @param pac the packet reader to make a decoder from
     * @return the Part Pointer for the data from the packet
     */
    PartPtr make_decoder();
    /**
     * @return the current byte the reader is on
     */
    uint8_t get_byte();
    /**
     * @return the type of the current byte the reader is on
     */
    Type get_type();
    /**
     * @return a string of bytes the reader is reading until the next 0 byte (end of the Packet)
     */
    std::string get_string();

    /**
     * @return the value stored by a Number Part
     */
    template <typename Number> Number get_number() {
        // ensures that the function is only used on numbers
        static_assert(
          std::is_floating_point<Number>::value || std::is_integral<Number>::value,
          "This function should only be used on numbers"
        );
        // checks that the size of the number its trying to read combined with its location
        // doesnt put it past the packet size
        if (read_head + sizeof(Number) > pac.size()) {
            printf(
              "%s:%d: Reading a number[%d] at position %d would read past "
              "buffer of "
              "size %d\n",
              __FILE__, __LINE__, sizeof(Number), read_head, pac.size()
            );
            return 0;
        }
        Number value = 0;
        // copies the the number at the reader head to the Number's stored value and
        // adds the size of the number to the read head so it moves on to the next set of bits
        std::memcpy(&value, &pac[read_head], sizeof(Number));
        read_head += sizeof(Number);
        return value;
    }

  private:
    Packet pac;
    size_t read_head;
};
///////////////////////////////// Packet Writer /////////////////////////////////
class PacketWriter {
  public:
    /**
     * creates a packet writer
     * @param scratch_space the packet for the writer to write to
     */
    explicit PacketWriter(Packet &scratch_space);
    /**
     * clears the packet the writer is writing to
     */
    void clear();
    /**
     * @return the size of the packet
     */
    size_t size();
    /**
     * writes a byte to the end of the packet
     * @param b the byte to write
     */
    void write_byte(uint8_t b);
    /**
     * writes a VDP type to the packet in the form of a byte
     * @param t the VDP type to write to the packet
     */
    void write_type(Type t);
    /**
     * writes a string to the packet
     * @param str the string to write to the packet
     */
    void write_string(const std::string &str);
    /**
     * writes a broadcast acknowledgement of a channel to the packet
     * @param chan the channel to write the acknowledgement for
     */
    void write_channel_acknowledge(const Channel &chan);
    /**
     * writes a broadcast of a channel schematic to the packet
     * @param chan the channel to write the schematic from
     */
    void write_channel_broadcast(const Channel &chan);
    /**
     * writes a response packet to the packets
     * @param chan the Channel to write the data from
     */
    void write_response(std::deque<Channel> &channels);
    /**
     * writes a broadcast of a channel schematic to the packet
     * @param chan the channel to request
     */
    void write_data_message(const Channel &part);
    /**
     * writes a request for a channel schematic to the packets
     * @param chan the Channel to write the data from
     */
    void write_request();
    /**
     * @return the packet the writer is writing to
     */
    const Packet &get_packet() const;
    /**
     * writes a number to the end of the packet
     */
    template <typename Number> void write_number(const Number &num) {
        std::array<uint8_t, sizeof(Number)> bytes;
        std::memcpy(&bytes, &num, sizeof(Number));
        for (const uint8_t b : bytes) {
            write_byte(b);
        }
    }

  private:
    Packet &sofar;
};

}