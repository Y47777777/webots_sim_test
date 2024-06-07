/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:08
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 11:30:21
 * @FilePath: /webots_ctrl/include/controller/base_ctrl.h
 * @Description:  ctrler base 类
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */

#ifndef __BASE_CTRL_H__
#define __BASE_CTRL_H__

#include <vector>
#include <map>
#include <QThread>
#include <thread>
#include "time/time.h"

#include "webots_device/w_base.h"

namespace VNSim {

class BaseController : public QThread {
   public:
    explicit BaseController(QObject *parent = nullptr) : QThread{parent} {
        supervisor_ = WSupervisor::getSupervisorInstance();
        step_duration_ = supervisor_->getBasicTimeStep();

        // init 设置为最快的模式
        supervisor_->step(step_duration_);
        supervisor_->simulationSetMode(Supervisor::SIMULATION_MODE_FAST);

        // 初始化timer
        timer_ptr_ = Timer::getInstance();
    }

    // 由base管理线程
    ~BaseController() {
        for (auto it = m_thread_.begin(); it != m_thread_.end(); ++it) {
            std::thread &thread = it->second;
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

    // 控制菜单相关，每个车型需要自己实现菜单控制相关部分
    virtual void manualGetState(std::map<std::string, double> &msg) = 0;
    virtual void manualSetState(const std::map<std::string, double> &msg) = 0;
    void shiftControlMode(bool mode) { isManual_ = mode; }

   protected:
    // ctrl while 循环
    void run() {
        // init
        if (supervisor_ == nullptr) {
            LOG_ERROR("Supervisor is nullptr");
            return;
        }

        // task
        webotsExited_ = false;
        alarm_.alarmTimerInit(step_duration_);

        while (supervisor_->step(step_duration_) != -1) {
            // 循环遍历注册的任务
            for (int i = 0; i < v_while_spin_.size(); ++i) {
                v_while_spin_[i]();
            }

            // 特殊的需要执行的任务
            this->whileSpin();
            
            // 休眠直到目标时间
            alarm_.wait();
        }
        webotsExited_ = true;

        // exit TODO:正常退出
        // supervisor_->simulationQuit(EXIT_SUCCESS);
        // delete supervisor_;
    }

    // spin task
    // 机器差异部分在该函数下实现
    virtual void whileSpin() = 0;

   protected:
    bool isManual_ = false;
    // webots
    WSupervisor *supervisor_ = nullptr;
    int step_duration_ = 0;
    bool webotsExited_ = false;

    std::map<std::string, std::thread> m_thread_;
    std::vector<std::function<void(void)>> v_while_spin_;
    std::shared_ptr<Timer> timer_ptr_;
    Timer alarm_;
};

}  // namespace VNSim

#endif
