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
            "../../../../../AGVServices/general/config/"
            "Actuators.config";  // use General config files...
        const char *encoder_file =
            "../../../../../AGVServices/general/config/"
            "Sencers.config";  // use General config files...
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
            // receive msg from general
            ecal_ptr_->addEcal(
                "Actuator/write",
                std::bind(
                    [&](const char *topic_name,
                        const eCAL::SReceiveCallbackData *data) {
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
            // report thread
            // std::thread sensor_msg_report_thread([&]() {
            //     Timer alarm;
            //     alarm.alarmTimerInit(10);
            //     while (!SVCExit_) {
            //         this->pubUpStream();
            //         alarm.wait();
            //     }
            // });
            // sensor_msg_report_thread_ = std::move(sensor_msg_report_thread);
        } while (0);
        return ret;
    }

   public:
    virtual int onInitService() = 0;
    virtual void subDownStreamCallBack(uint8_t *msg, int len) = 0;
    virtual void pubUpStream() = 0;

   protected:
    InputDecoder decoder_;
    OutputEncoder encoder_;
    std::mutex msgs_lock_;

    // 发送的包数
    uint32_t dataidx_upload_ = 0;
    uint32_t dataidx_sub_ = 0;
    bool first_pub_report_ = true;

};
}  // namespace VNSim

#endif