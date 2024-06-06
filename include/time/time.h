#ifndef __TIME_H__
#define __TIME_H__
#include <chrono>
#include <thread>
#include <qelapsedtimer.h>

namespace VNSim {
class FixedTimeWakeUpTimer {
   private:
    uint32_t duration_;  // ms
    std::chrono::system_clock::time_point next_wakeup_;

   public:
    FixedTimeWakeUpTimer() {}
    ~FixedTimeWakeUpTimer() {}

   public:
    void ready(uint32_t duration) {
        duration_ = duration;
        next_wakeup_ = std::chrono::system_clock::now();
    }
    void wait() {
        next_wakeup_ += (std::chrono::microseconds(duration_ * 1000));
        std::this_thread::sleep_until(next_wakeup_);
    }
};

class FixedTimeTimestampGenerator {
   private:
    uint64_t next_timestamp_;
    uint32_t duration_;  // ms

   public:
    FixedTimeTimestampGenerator(uint32_t duration = 10,
                                uint64_t base_timestamp = (1640966400000000 +
                                                           8 * 3600 + 1000000))
        : next_timestamp_(base_timestamp), duration_(duration) {}
    ~FixedTimeTimestampGenerator() {}
    uint64_t timestamp() {
        uint64_t l_timestamp = next_timestamp_;
        next_timestamp_ += (duration_ * 1000);
        return l_timestamp;
    }
};

class ElapsedTimer {
   private:
    QElapsedTimer timer_;
    uint64_t next_timestamp_;

   public:
    void start(uint64_t base_timestamp = (1640966400000000 + 8 * 3600 +
                                          1000000)) {
        next_timestamp_ = base_timestamp;
        timer_.start();
        return;
    }
    void restart() {
        timer_.restart();
        return;
    }
    uint64_t elapsed() {
        if (!timer_.isValid()) {
            return 0;
        }
        uint64_t l_elapsed = timer_.elapsed();
        next_timestamp_ += (l_elapsed * 1000);
        return l_elapsed;
    }
    uint64_t timestamp() { return next_timestamp_; }
};

class Timer {
   public:
    static std::shared_ptr<Timer> getInstance() {
        if (static_timer_ptr_ == nullptr) {
            static_timer_ptr_ = std::make_shared<Timer>();
            static_timer_ptr_->setBaseTime();
        }
        return static_timer_ptr_;
    }

    void alarmTimerInit(int duration) {
        step_duration_ = duration;
        start_ = std::chrono::high_resolution_clock::now();
        next_wakeup_ =
            start_ + (std::chrono::microseconds(step_duration_ * 1000));
    }

    // please init alarm before use
    void wait() {
        std::this_thread::sleep_until(next_wakeup_);
        next_wakeup_ += (std::chrono::microseconds(step_duration_ * 1000));
    }

    void restart() { start_ = std::chrono::high_resolution_clock::now(); }

    template <typename T>
    uint32_t elapsed() {
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<T>(end - start_);
        return elapsed.count();
    }

    void setBaseTime(uint64_t base = 1640966400000000 +
                                     (long long) 8 * 3600 * 1000000) {
        start_ = std::chrono::high_resolution_clock::now();
        base_time = std::chrono::microseconds(base);
    }

    template <typename T>
    uint64_t now() {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto time = std::chrono::duration_cast<T>(duration);

        return (time.count());
    }

    uint64_t getTimeStamp() {
        auto now = std::chrono::system_clock::now();

        auto time = (now - start_) + base_time;
        auto time_stamp =
            std::chrono::duration_cast<std::chrono::microseconds>(time);

        return time_stamp.count();
    }

   private:
    std::chrono::high_resolution_clock::time_point start_;
    std::chrono::microseconds base_time;

    std::chrono::system_clock::time_point next_wakeup_;
    int step_duration_ = 0;

    static std::shared_ptr<Timer> static_timer_ptr_;
};
}  // namespace VNSim

#endif