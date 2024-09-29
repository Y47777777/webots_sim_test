/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-07 17:39:39
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 20:05:13
 * @FilePath: /webots_ctrl/include/svc/base_svc_serial.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#ifndef __BASE_SERIAL_SVC_SERIAL_H__
#define __BASE_SERIAL_SVC_SERIAL_H__
#include <thread>
#include <shared_mutex>
#include <semaphore.h>
#include "logvn/logvn.h"
#include "Parser/parser.hpp"
#include "time/time.h"
#include "lock/Spinlock.h"
#include "base_svc.h"

namespace VNSim {

class BaseSerialSvc : public BaseSvc {
   public:
    BaseSerialSvc() : BaseSvc() {}
    ~BaseSerialSvc() {}

   protected:
    // TODO: config 路径修改
    int initService() {
        const char *decoder_file =
            "/home/visionnav/AGVServices/general/config/Actuators.config";  // use General config files...
        const char *encoder_file =
            "/homve/visionnav/AGVServices/general/config/Sencers.config";  // use General config files...
        int ret = 0;
        do { /*load config*/
            int l_ret = decoder_.loadConfig(decoder_file);
            if (l_ret != 0) {
                LOG_WARN("decoder loadConfig ret = %d", l_ret);
                ret = -1;
                break;
            }
            l_ret = encoder_.loadConfig(encoder_file);
            if (l_ret != 0) {
                LOG_WARN("encoder loadConfig ret = %d", l_ret);
                ret = -2;
                break;
            }
            l_ret = sem_init(&watch_dog_sem_, 0, 0);
            // receive msg from general
            ecal_ptr_->addEcal(
                "Actuator/write",
                std::bind(
                    [&](const char *topic_name,
                        const eCAL::SReceiveCallbackData *data) {
                        if(!this->feedDog()){
                            LOG_WARN("Dog is currently hungry..., won't trigger...");
                            return;
                        }
                        struct Package pack {
                            (uint8_t *) data->buf, (int) data->size
                        };
                        this->subDownStreamCallBack(
                            pack.buf,
                            pack.len);  // user send webot
                                        // in the derive class
                    },
                    std::placeholders::_1, std::placeholders::_2));
            this->onInitService();
            // timer thread
            std::thread msg_watch_dog([&]() {
                Timer alarm;
                while(!SVCExit_){
                    LOG_INFO("msg watch dog thread ready...");
                    //printf("msg watch dog thread ready...\n");
                    sem_wait(&watch_dog_sem_);
                    LOG_INFO("msg watch dog thread trigger...");
                    //printf("msg watch dog thread trigger...\n");
                    feed_dog_mutex_.lock();
                    msg_watch_dog_ready_ = true;
                    feed_dog_food_ = 0;
                    isDogHungry_ = false;
                    feed_dog_mutex_.unlock();
                    while(!SVCExit_){
                        //alarm.restart();
                        alarm.alarmTimerInit(1500);
                        alarm.wait();
                        feed_dog_mutex_.lock();
                        if(feed_dog_food_ == 0){
                            // timeout
                            msg_watch_dog_ready_ = false;
                            isDogHungry_ = true;
                            feed_dog_mutex_.unlock();
                            LOG_WARN("%s --> dog is hungry, need hot dog...", __FUNCTION__);
                            //printf("%s --> dog is hungry, need hot dog...\n", __FUNCTION__);
                            this->onWatchDogHungry();
                            break;
                        }else{
                            feed_dog_food_ = 0;
                            feed_dog_mutex_.unlock();
                        }
                    }
                }
            });
            msg_watch_dog_ = std::move(msg_watch_dog);
        } while (0);
        return ret;
    }

   public:
    virtual int onInitService() = 0;
    virtual void subDownStreamCallBack(uint8_t *msg, int len) = 0;
    virtual void pubUpStream() = 0;
    virtual void onWatchDogHungry(){

    }
   public:
    bool feedDog(){
        bool isFeedDogOk = true;
        bool isNeedPost = false;
        feed_dog_mutex_.lock();
        if(msg_watch_dog_ready_ == false){
            // first, wont feed dog, only start timer
            if(!is_post_){
                is_post_ = true;
                isNeedPost = true;
            }
            feed_dog_mutex_.unlock();
            if(isNeedPost){
                LOG_INFO("%s --> sem_post reset dog...", __FUNCTION__);
                sem_post(&watch_dog_sem_);
            }
        }else{
            is_post_ = false;
            if(!isDogHungry_){
                feed_dog_food_++;
            }else{
                isFeedDogOk = false;
            }
            feed_dog_mutex_.unlock();
        }
        return isFeedDogOk;
    }
   protected:
    InputDecoder decoder_;
    OutputEncoder encoder_;
    std::mutex msgs_lock_;

    // 发送的包数
    uint32_t dataidx_upload_ = 0;
    uint32_t dataidx_sub_ = 0;
    bool first_pub_report_ = true;
    
   private:
    sem_t watch_dog_sem_;
    std::mutex watch_dog_signal_mutex_;
    std::mutex feed_dog_mutex_;
    std::thread msg_watch_dog_;
    uint64_t feed_dog_food_{0};
    bool isDogHungry_{false};
    bool msg_watch_dog_ready_{false};
    bool is_post_{false};

};
}  // namespace VNSim

#endif