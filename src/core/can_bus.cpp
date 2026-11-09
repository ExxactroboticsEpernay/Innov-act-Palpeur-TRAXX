#include "can_bus.h"

namespace palpeur {

bool DisabledCanBus::begin(std::uint32_t) { return false; }

bool DisabledCanBus::send(const CanFrame&) { return false; }

bool DisabledCanBus::receive(CanFrame&) { return false; }

bool DisabledCanBus::enabled() const { return false; }

}  // namespace palpeur