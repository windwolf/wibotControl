//
// Created by zhouj on 2023/8/11.
//

#ifndef AQ_SPXX_RC_LIBS_WIBOTCONTROL_FSMLITE_HPP_
#define AQ_SPXX_RC_LIBS_WIBOTCONTROL_FSMLITE_HPP_

#include "base.hpp"
#include "eventgroup.hpp"

namespace wibot::control {

typedef uint32_t FsmState;

using FsmOnStateChangedCallback         = void (*)(FsmState old_state, FsmState new_state);
using FsmOnStateEnterCallback           = void (*)(FsmState state);
using FsmOnStateExitCallback            = void (*)(FsmState state);
using FsmOnStatePollingCallback         = void (*)(FsmState state);
using FsmOnStateTransitionCheckCallback = FsmState (*)(FsmState& new_state);

class FsmLite {
   public:
    FsmLite(void* userData, FsmOnStateTransitionCheckCallback onStateTransitionCheck,
            FsmOnStateChangedCallback onStateChanged, FsmOnStatePollingCallback onStatePolling);

    /**
      * 检查是否需要状态迁移。一般是低频检查。
      * @param tick
      */
    void check(uint32_t tick);

    /**
      * 实施实际变更工作，并触发对应动作。一般是高频检查。
      */
    void update(uint32_t tick);

    /**
      * 获取当前状态持续时间
      */
    uint32_t get_current_state_duration() const {
        return current_tick - current_state_entry_tick;
    }

    /**
      * 获取当前状态
      * @return
      */
    FsmState get_current_state() const {
        return current_state;
    }

    /**
      * 获取上一个状态
      * @return
      */
    FsmState get_last_state() const {
        return last_state;
    }

   protected:
    FsmState new_state;
    FsmState current_state;
    FsmState last_state;

    uint32_t current_tick;
    uint32_t current_state_entry_tick;
    uint32_t last_state_entry_tick;

    FsmOnStateTransitionCheckCallback on_state_transition_check;
    FsmOnStateChangedCallback         on_state_changed;
    FsmOnStatePollingCallback         on_state_polling;

    void* userData;
};

}  // namespace wibot::control

#endif  //AQ_SPXX_RC_LIBS_WIBOTCONTROL_FSMLITE_HPP_
