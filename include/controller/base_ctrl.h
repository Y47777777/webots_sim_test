#ifndef __BASE_CTRL_H__
#define __BASE_CTRL_H__

#include <vector>
#include <map>
#include <QThread>
#include <thread>
#include "time/time.h"

#include "webots_device/w_base.h"

// TODO: 整理成time.hpp
#include <chrono>
#include <QElapsedTimer>
#include <QTime>

namespace VNSim {

class BaseController : public QThread {
   public:
    explicit BaseController(QObject *parent = nullptr) : QThread{parent} {
        supervisor_ = WSupervisor::getSupervisorInstance();
        step_duration_ = supervisor_->getBasicTimeStep();

        // init 设置为最快的模式
        supervisor_->step(step_duration_);
        supervisor_->simulationSetMode(Supervisor::SIMULATION_MODE_FAST);
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
        wakeup_timer_.ready(step_duration_);

        // FIXME: for test
        // QElapsedTimer t;
        // QElapsedTimer t_1;
        // QElapsedTimer sleep_t;
        // LOG_INFO("                         step %d", t_1.elapsed());
        // t.restart();
        // LOG_INFO("copy spend %d", t.elapsed());
        // sleep_t.restart();
        // LOG_INFO("sleep %d", sleep_t.elapsed());
        // t_1.restart();

        while (supervisor_->step(step_duration_) != -1) {
            // 循环遍历注册的任务
            for (int i = 0; i < v_while_spin_.size(); ++i) {
                v_while_spin_[i]();
            }

            // 特殊的需要执行的任务
            this->whileSpin();

            // 休眠直到目标时间
            wakeup_timer_.wait();
        }
        webotsExited_ = true;

        // exit TODO:正常退出
        // supervisor_->simulationQuit(EXIT_SUCCESS);
        // delete supervisor_;
    }

    // spin task
    virtual void whileSpin() = 0;

   protected:
    bool isManual_ = false;
    // webots
    WSupervisor *supervisor_ = nullptr;
    int step_duration_ = 0;
    FixedTimeWakeUpTimer wakeup_timer_;
    bool webotsExited_ = false;

    std::map<std::string, std::thread> m_thread_;
    std::vector<std::function<void(void)>> v_while_spin_;
};

}  // namespace VNSim

#endif
