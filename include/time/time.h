#ifndef __TIME_H__
#define __TIME_H__
#include <chrono>
#include <thread>

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
                                uint64_t base_timestamp = 1640966400000000)
        : next_timestamp_(base_timestamp), duration_(duration) {}
    ~FixedTimeTimestampGenerator() {}
    uint64_t timestamp() {
        uint64_t l_timestamp = next_timestamp_;
        next_timestamp_ += (duration_ * 1000);
        return l_timestamp;
    }
};

#endif