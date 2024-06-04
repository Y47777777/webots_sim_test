#ifndef __BASE_SERIAL_SVC_SERIAL_H__
#define __BASE_SERIAL_SVC_SERIAL_H__
#include <thread>
#include "logvn/logvn.h"
#include "Parser/parser.hpp"
#include "time/time.h"
#include "base_svc_ctrl.h"

namespace VNSim {

class BaseSerialSVCModel : public BaseSVCModel {
   protected:
    InputDecoder decoder_;
    OutputEncoder encoder_;
    uint64_t lidar_count_;
    std::thread sensor_msg_report_thread_;  // not lidar, sensors.config
   public:
    BaseSerialSVCModel() : BaseSVCModel(), lidar_count_(0) {}
    ~BaseSerialSVCModel() {
        if (sensor_msg_report_thread_.joinable()) {
            sensor_msg_report_thread_.join();
        }
    }

   protected:
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
            ecal_wrapper_.addEcal(
                "Actuator/write",
                std::bind(
                    [&](const char *topic_name,
                        const eCAL::SReceiveCallbackData *data) {
                        struct Package pack {
                            (uint8_t *) data->buf, (int) data->size
                        };
                        this->onDownStreamProcess(
                            pack.buf,
                            pack.len);  // user send webot
                                        // in the derive class
                    },
                    std::placeholders::_1, std::placeholders::_2));
            this->onInitService();
            // report thread
            std::thread sensor_msg_report_thread([&]() {
                FixedTimeWakeUpTimer wakeup_timer;
                wakeup_timer.ready(10);
                while (!SVCExit_) {
                    wakeup_timer.wait();
                    this->onUpStreamProcess();
                }
            });
            sensor_msg_report_thread_ = std::move(sensor_msg_report_thread);
        } while (0);
        return ret;
    }

   public:
    virtual int onInitService() = 0;
    virtual void onDownStreamProcess(uint8_t *msg, int len) = 0;
    virtual void onUpStreamProcess() = 0;
};  // namespace VNSim
}  // namespace VNSim

#endif