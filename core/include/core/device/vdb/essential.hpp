#pragma once
#include <sstream>
#include "core/device/vdb/packet.hpp"
#include "core/device/vdb/crc32.hpp"
#include "core/device/vdb/channel.hpp"
#include "core/device/vdb/visitor.hpp"
#include "core/device/vdb/types.hpp"
namespace VDP{

uint32_t time_ms();
void delay_ms(uint32_t ms);

#define VDPDEBUG
#define VDPWARN

#ifdef VDPWARN
#define VDPWarnf(fmt, ...) printf("WARN: " fmt "\n", ##__VA_ARGS__)
#else
#define VDPWarnf(...)
#endif

#ifdef VDPDEBUG
#define VDPDebugf(fmt, ...) printf("DEBUG: " fmt "\n", ##__VA_ARGS__)
#else
#define VDPDebugf(...)
#endif

#ifdef VDPTRACE
#define VDPTracef(fmt, ...) printf("TRACE: " fmt "\n", ##__VA_ARGS__)
#else
#define VDPTracef(...)
#endif

/**
 * adds indents to a stringstream
 * @param ss the stringstream to add indents to
 * @param indent the amount of double spaced indents to add
 */
void add_indents(std::stringstream &ss, size_t indent);

/** Returns a string of the value type from a packet */
std::string to_string(Type t);

}