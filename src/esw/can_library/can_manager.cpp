#include "can_manager.hpp"

std::vector<uint8_t> createFloatMessage(float floatValue) {
    FloatMessage u = {floatValue};
    std::vector<uint8_t> message(u.bytes, u.bytes + sizeof(float));
    return message;
}

std::vector<uint8_t> createBoolMessage(bool boolValue) {
    BoolMessage u = {boolValue};
    std::vector<uint8_t> message(u.bytes, u.bytes + sizeof(bool));
    return message;
}