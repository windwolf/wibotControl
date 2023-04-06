#ifndef ___FRAMEWORK_HPP__
#define ___FRAMEWORK_HPP__

#include "base/base.hpp"

#define CONTROL_NODE_ZISE_IN_SCOPE 10

namespace wibot::control {
using namespace std;

/**
 * framework要解决几个问题.
 * 1. 需支持各种不同带宽的控制环路
 * 2. 同一控制环路共享时间相关参数.
 * 3. 要提供参数更新机制, 有些控制参数不变, 有些变化频率非常低, 有些控制参数会变化很快.
 * 4.
 */

class ControlScope {
   public:
    enum class Mode {
        FixStep,
        VariableStep,
    };

   public:
    ControlScope(float sample_time) {
        _mode                = Mode::FixStep;
        sample_time          = sample_time;
        _sample_time_changed = false;
    }

    ControlScope() {
        _mode = Mode::VariableStep;
    }

    void update(float sample_time) {
        if (_mode == Mode::FixStep) {
            _sample_time_changed = false;
            return;
        }
        if (sample_time != _sample_time) {
            _sample_time         = sample_time;
            _sample_time_changed = true;
        } else {
            _sample_time_changed = false;
        }
    };

    float sample_time_get() {
        return _sample_time;
    }
    bool sample_time_changed() {
        return _sample_time_changed;
    }

   private:
    Mode     _mode;
    float    _sample_time;
    uint32_t _tick;
    bool     _sample_time_changed;
};

class ControlNode {
   public:
    friend class ControlScope;

    void init();

    virtual void update(ControlScope& scp) = 0;
};

}  // namespace wibot::control
#endif  // ___FRAMEWORK_HPP__
