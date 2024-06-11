/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-07 15:57:36
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 17:28:48
 * @FilePath: /webots_ctrl/include/data_process_service/process_service_base.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once
#include <thread>
#include <shared_mutex>
#include <stdint.h>
#include <memory>

#include "Parser/parser.hpp"
#include "logvn/logvn.h"
#include "Parser/parser.hpp"
#include "time/time.h"
#include "ecal_wrapper/ecal_wrapper.h"

namespace VNSim {

class CtrlProcessBase {
   public:
    CtrlProcessBase() { init(); }
    ~CtrlProcessBase() {
        for (auto it = m_thread_.begin(); it != m_thread_.end(); ++it) {
            std::thread &thread = it->second;
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

   protected:
    // 与固件的通讯 由base触发
    virtual void fromCtrlSystem(uint8_t *msg, int len) = 0;
    virtual void toCtrlSystem() = 0;

    // 与仿真系统的通讯 由base触发
    virtual void fromSimSystem(uint8_t *msg, int len) = 0;
    virtual void toSimSystem() = 0;

   protected:
    InputDecoder decoder_;
    OutputEncoder encoder_;
    std::shared_ptr<EcalWrapper> ecal_ptr_;
    std::map<std::string, std::thread> m_thread_;
    bool task_is_exit_ = false;

   private:
    void init() {
        ecal_ptr_ = EcalWrapper::getInstance("simulation_process");
        initCoder();

        // creat down stream process
        ecal_ptr_->addEcal("Actuator/write",
                           std::bind(
                               [&](const char *topic_name,
                                   const eCAL::SReceiveCallbackData *data) {
                                   struct Package pack {
                                       (uint8_t *) data->buf, (int) data->size
                                   };
                                   this->fromCtrlSystem(
                                       pack.buf,
                                       pack.len);  // user send webot
                                                   // in the derive class       
                               },
                               std::placeholders::_1, std::placeholders::_2));

        // creat up sream process
        m_thread_["up_stream_process"] = std::thread([&]() {
            Timer alarm;
            alarm.alarmTimerInit(10);
            while (!task_is_exit_) {
                this->toCtrlSystem();
                alarm.wait();
            }
        });
    }

    int initCoder() {
        const char *decoder_file =
            "../../../../../AGVServices/general/config/"
            "Actuators.config";  // use General config files...
        const char *encoder_file =
            "../../../../../AGVServices/general/config/"
            "Sencers.config";  // use General config files...
        int l_ret = decoder_.loadConfig(decoder_file);
        if (l_ret != 0) {
            LOG_WARN("decoder loadConfig ret = %d", l_ret);
            return -1;
        }
        l_ret = encoder_.loadConfig(encoder_file);
        if (l_ret != 0) {
            LOG_WARN("encoder loadConfig ret = %d", l_ret);
            return -2;
        }
    }
};

}  // namespace VNSim