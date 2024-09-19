/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:08
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 10:40:23
 * @FilePath: /webots_ctrl/include/webots_device/w_base.h
 * @Description:
 *                  webots 接口base
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once

#include <webots/Supervisor.hpp>
#include <shared_mutex>
#include "logvn/logvn.h"
#include "atomic_lock.h"

namespace VNSim {
using namespace webots;

class WSupervisor : public Supervisor {
   public:
    /**
     * @brief Get the Supervisor Instance object
     *
     * @return WSupervisor*
     */
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

    /**
     * @brief webots step
     *        webots 数据生成在step函数中完成，读取数据应该在step中完成
     *
     * @param[in] duration step 的步长(ms) webots执行一步
     * @return int
     */
    int step(int duration) {
        // 计算方针总步数
        step_cnt++;
        return Supervisor::step(duration);
    }

    /**
     * @brief Get the Step Cnt object
     *
     * @return uint64_t
     */
    uint64_t getStepCnt() { return step_cnt; }

   private:
    WSupervisor() : Supervisor() {}
    uint64_t step_cnt = 0;
};

class WBase {
   public:
    WBase() {
        // 获取supervisor basetime
        super_ = WSupervisor::getSupervisorInstance();
        step_duration_ = super_->getBasicTimeStep();
    }
    ~WBase() {}

    virtual void spinWithAtomicMutex(void) {
        AutoAtomicLock lock(spin_mutex_);
        spin();
    }

    virtual void spin() = 0;

   protected:
    int step_duration_;
    int start_step_ = 0;
    WSupervisor *super_;
    std::shared_mutex rw_mutex_;  // 读写锁
    SpinLock spin_mutex_;
};
}  // namespace VNSim