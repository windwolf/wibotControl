//
// Created by zhouj on 2023/8/11.
//

#include "fsmlite.hpp"

namespace wibot::control {
void FsmLite::update(uint32_t tick) {
    current_tick = tick;
    if (new_state != current_state) {
        last_state               = current_state;
        current_state            = new_state;
        last_state_entry_tick    = current_state_entry_tick;
        current_state_entry_tick = current_tick;
        on_state_changed(last_state, current_state);
    }
    on_state_polling(current_state);
};

void FsmLite::check(uint32_t tick) {
    current_tick = tick;
    FsmState newSta;
    if (on_state_transition_check(newSta)) {
        this->new_state = newSta;
    }
}

FsmLite::FsmLite(void* userData, FsmOnStateTransitionCheckCallback onStateTransitionCheck,
                 FsmOnStateChangedCallback onStateChanged, FsmOnStatePollingCallback onStatePolling)
    : on_state_transition_check(onStateTransitionCheck),
      on_state_changed(onStateChanged),
      on_state_polling(onStatePolling),
      userData(userData){

      };

}  // namespace wibot::control
