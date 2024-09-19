/**
 * @file atomic_lock.h
 * @author weijianchen (weijianchen@visionnav.com)
 * @brief  原子锁
 * @version 0.1
 * @date 2024-09-19
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <shared_mutex>
#include "logvn/logvn.h"

class SpinLock {
   public:
    void lock() {
        while (flag.test_and_set(std::memory_order_acquire)) {}
    }

    void unlock() { flag.clear(std::memory_order_release); }

   private:
    std::atomic_flag flag = ATOMIC_FLAG_INIT;
};

/**
 * @brief 自动原子锁
 *
 */
class AutoAtomicLock {
   public:
    AutoAtomicLock(SpinLock &mutex) : atomicLock_(mutex) { atomicLock_.lock(); }
    ~AutoAtomicLock() { atomicLock_.unlock(); }

   private:
    SpinLock &atomicLock_;
};