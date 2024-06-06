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
}  // namespace VNSim

#endif