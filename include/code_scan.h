#ifndef __CODE_SCAN_H__
#define __CODE_SCAN_H__
#include <condition_variable>
namespace VNSim {
#define SCAN_TIME 6000  // 6s
class CoderNotifyer {
   private:
    std::condition_variable code_notify;
    std::mutex mtx_;

   public:
    CoderNotifyer() {}
    ~CoderNotifyer() {}
    void onCode() { code_notify.notify_one(); }
    void wait(int time = SCAN_TIME) {
        std::unique_lock<std::mutex> lk(mtx_);
        code_notify.wait_for(lk, std::chrono::milliseconds(time));
    }
};

class CoderManager {
   public:
    enum STATE_CODE { IDLE = 0, STANDBY, RUNNING };

   private:
    std::shared_ptr<CoderNotifyer> listener_;
    std::condition_variable running_notify;
    std::mutex wait_mutex1_;
    std::mutex wait_mutex2_;
    std::mutex running_mutex_;
    bool running_{false};
    bool trigger_next_{false};
    int state_{STATE_CODE::STANDBY};

   public:
    CoderManager(std::shared_ptr<CoderNotifyer> ptr) { listener_ = ptr; }
    ~CoderManager() {}
    int standBy() {
        int ret = 0;
        std::unique_lock<std::mutex> lk(wait_mutex1_);
        {
            running_notify.wait(lk, [&]() { return trigger_next_; });
            trigger_next_ = false;
        }
        running_mutex_.lock();
        if (!running_)
            ret = -1;
        running_mutex_.unlock();
        return ret;
    }
    int run() {
        int ret = 1;
        wait_mutex2_.lock();
        state_ = STATE_CODE::RUNNING;
        wait_mutex2_.unlock();
        listener_->wait();
        wait_mutex2_.lock();
        if (state_ != STATE_CODE::STANDBY) {
            ret = 0;
            state_ = STATE_CODE::STANDBY;
        }
        wait_mutex2_.unlock();
        return ret;
    }
    void start_scan() {
        wait_mutex2_.lock();
        int state_l = state_;
        wait_mutex2_.unlock();
        running_mutex_.lock();
        running_ = true;
        running_mutex_.unlock();
        if (state_l == int(STATE_CODE::STANDBY)) {
            std::unique_lock<std::mutex> lk(wait_mutex1_);
            {
                trigger_next_ = true;
                lk.unlock();
            }
            running_notify.notify_one();
        }
    }
    void stop_scan() {
        wait_mutex2_.lock();
        state_ = STATE_CODE::STANDBY;
        wait_mutex2_.unlock();
        listener_->onCode();
    }
    void stop() {
        wait_mutex2_.lock();
        state_ = STATE_CODE::STANDBY;
        wait_mutex2_.unlock();
        running_mutex_.lock();
        running_ = false;
        running_mutex_.unlock();
        std::unique_lock<std::mutex> lk(wait_mutex1_);
        {
            trigger_next_ = true;
            lk.unlock();
        }
        listener_->onCode();
        running_notify.notify_one();
    }
};  // namespace VNSim
}  // namespace VNSim
#endif