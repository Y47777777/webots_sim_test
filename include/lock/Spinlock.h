#ifndef __SPINLOCK_H__
#define __SPINLOCK_H__
#include <stdbool.h>
#include <atomic>

namespace VNSim {
class Spinlock {
   public:
    Spinlock() {}
    Spinlock(const Spinlock &) = delete;
    Spinlock &operator=(const Spinlock &) = delete;
    void lock() {
        while (flag.test_and_set(std::memory_order_acquire))
            ;
    }
    void unlock() { flag.clear(std::memory_order_release); }

   private:
    std::atomic_flag flag{ATOMIC_FLAG_INIT};
};
}  // namespace VNSim
#endif