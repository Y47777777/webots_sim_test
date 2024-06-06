#pragma once

#include <webots/Supervisor.hpp>
#include <shared_mutex>
#include "logvn/logvn.h"

namespace VNSim {
using namespace webots;
class WSupervisor : public Supervisor {
   public:
    static WSupervisor *getSupervisorInstance() {
        if (cInstance) {
            if (!dynamic_cast<WSupervisor *>(cInstance)) {
                LOG_ERROR("Supervisor is nullptr");
                return NULL;
            }
            return static_cast<WSupervisor *>(cInstance);
        }
        cInstance = new WSupervisor();
        return static_cast<WSupervisor *>(cInstance);
    }

    int step(int duration) {
        step_cnt++;
        return Supervisor::step(duration);
    }

    int getStepCnt() { return step_cnt; }

   private:
    WSupervisor() : Supervisor() {}
    int step_cnt = 0;
};

class WBase {
   public:
    WBase() {
        super_ = WSupervisor::getSupervisorInstance();
        step_duration_ = super_->getBasicTimeStep();
    }
    ~WBase() {}

    virtual void spin() = 0;

   protected:
    int step_duration_;
    int start_step_ = 0;
    WSupervisor *super_;
    std::shared_mutex rw_mutex_;  // 读写锁
};
}  // namespace VNSim