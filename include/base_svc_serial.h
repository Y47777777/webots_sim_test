#ifndef __BASE_SERIAL_SVC_SERIAL_H__
#define __BASE_SERIAL_SVC_SERIAL_H__
#include <thread>
#include "Parser/parser.hpp"
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
        // TODO: may be it is not exection path...,
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
                ret = -1;
                break;
            }
            l_ret = encoder_.loadConfig(encoder_file);
            if (l_ret != 0) {
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
                auto last_check = std::chrono::system_clock::now();
                while (!SVCExit_) {
                    auto now_t = std::chrono::system_clock::now();
                    auto duration_t =
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            now_t - last_check);
                    if (duration_t.count() >= 10) {
                        last_check = std::chrono::system_clock::now();
                        this->onUpStreamProcess();
                    } else {
                        // keep reporting rate at 100HZ idealy
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(10 - duration_t.count()));
                    }
                }
            });
            sensor_msg_report_thread_ = std::move(sensor_msg_report_thread);
        } while (0);
        std::cout << "ret = " << ret << std::endl;
        return ret;
    }

   public:
    virtual int onInitService() = 0;
    virtual void onDownStreamProcess(uint8_t *msg, int len) = 0;
    virtual void onUpStreamProcess() = 0;

   public:
    uint64_t getLidarCount() { return lidar_count_; }
};  // namespace VNSim
}  // namespace VNSim

#endif