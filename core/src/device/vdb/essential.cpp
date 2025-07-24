#include "core/device/vdb/essential.hpp"

namespace VDP{
/**
 * adds indents to a stringstream
 * @param ss the stringstream to add indents to
 * @param indent the amount of double spaced indents to add
 */
void add_indents(std::stringstream &ss, size_t indent) {
    for (size_t i = 0; i < indent; i++) {
        ss << "  ";
    }
}

/**
 * @param t the VDP type to return a string of
 * @return a string of the VDP type
 */
std::string to_string(Type t) {
    switch (t) {
    case Type::Record:
        return "record";
    case Type::String:
        return "string";

    case Type::Float:
        return "float";
    case Type::Double:
        return "double";

    case Type::Uint8:
        return "uint8";
    case Type::Uint16:
        return "uint16";
    case Type::Uint32:
        return "uint32";
    case Type::Uint64:
        return "uint64";

    case Type::Int8:
        return "int8";
    case Type::Int16:
        return "int16";
    case Type::Int32:
        return "int32";
    case Type::Int64:
        return "int64";
    }

    return "<<UNKNOWN TYPE>>";
}
}