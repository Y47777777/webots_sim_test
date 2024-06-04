#pragma once

#include <webots/Supervisor.hpp>
#include <shared_mutex>
#include "logvn/logvn.h"

namespace VNSim {
using namespace webots;
class WBase {
   public:
    WBase() {
        super_ = Supervisor::getSupervisorInstance();
        if (super_ == nullptr) {
            LOG_ERROR("Supervisor is nullptr");
        }
    }
    ~WBase() {}

    virtual void spin() = 0;

   protected:
    Supervisor *super_;
    std::shared_mutex rw_mutex_;  // 读写锁
};
}  // namespace VNSim