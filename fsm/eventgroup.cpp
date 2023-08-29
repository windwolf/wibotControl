#include "eventgroup.hpp"

namespace wibot::control {
bool FSM_EventGroup::check(FSM_EventFlag &eventFlags) {
    return (_events & eventFlags.mask) == (eventFlags.value & eventFlags.mask);
};
void FSM_EventGroup::set(uint32_t events) {
    _events |= events;
};
void FSM_EventGroup::reset(uint32_t events) {
    _events &= ~events;
};
void FSM_EventGroup::clear() {
    _events &= ~_eventsClearMask;
}
uint32_t FSM_EventGroup::get() {
    return _events;
};
}  // namespace wibot::control
