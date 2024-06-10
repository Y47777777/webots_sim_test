/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 16:12:38
 * @FilePath: /webots_ctrl/include/time/time.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */

#ifndef __TIME_H__
#define __TIME_H__
#include <chrono>
#include <thread>
#include <qelapsedtimer.h>

namespace VNSim {

// 定义类型别名
using seconds = std::chrono::seconds;
using milliseconds = std::chrono::milliseconds;
using microseconds = std::chrono::microseconds;
class Timer {
   public:
    /**
     * @brief Get the Instance object
     *
     * @return std::shared_ptr<Timer>
     */
    static std::shared_ptr<Timer> getInstance() {
        if (instance_ptr_ == nullptr) {
            instance_ptr_ = std::make_shared<Timer>();
            instance_ptr_->setBaseTime();
        }
        return instance_ptr_;
    }

    /**
     * @brief  闹钟初始化
     *
     * @param [duration]  每步休眠时间
     */
    void alarmTimerInit(int duration) {
        step_duration_ = duration;
        start_ = std::chrono::high_resolution_clock::now();
        next_wakeup_ = start_ + (microseconds(step_duration_ * 1000));
    }

    /**
     * @brief  等待闹钟唤醒
     *         使用前先初始化闹钟
     */
    void wait() {
        std::this_thread::sleep_until(next_wakeup_);
        next_wakeup_ += (microseconds(step_duration_ * 1000));
    }

    template <typename T>
    void sleep(uint64_t time) {
        std::this_thread::sleep_for(T(time));
    }

    /**
     * @brief  获取时间戳
     *
     * @return 时间戳(微秒)，可以通过 setBaseTime设置起点
     */
    uint64_t getTimeStamp() {
        auto now = std::chrono::system_clock::now();

        auto time = (now - start_) + base_time;
        auto time_stamp = std::chrono::duration_cast<microseconds>(time);

        return time_stamp.count();
    }

    /**
     * @brief  设置时间戳起点
     *
     * @param[base]  起点，默认起点 //beijing time : 2022.01.01 00:00:00; us
     */
    void setBaseTime(uint64_t base = 1640966400000000 +
                                     (long long) 8 * 3600 * 1000000) {
        start_ = std::chrono::high_resolution_clock::now();
        base_time = microseconds(base);
    }

    void restart() { start_ = std::chrono::high_resolution_clock::now(); }

    template <typename T>
    uint32_t elapsed() {
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<T>(end - start_);
        return elapsed.count();
    }

    template <typename T>
    uint64_t now() {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto time = std::chrono::duration_cast<T>(duration);

        return (time.count());
    }

   private:
    std::chrono::high_resolution_clock::time_point start_;
    microseconds base_time;

    std::chrono::system_clock::time_point next_wakeup_;
    int step_duration_ = 0;

    static std::shared_ptr<Timer> instance_ptr_;
};
}  // namespace VNSim

#endif