/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 12:30:40
 * @FilePath: /webots_ctrl/app/ST/svc_model_ST/svc_model_serial.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#ifndef __SVC_MODEL_SERIAL_H__
#define __SVC_MODEL_SERIAL_H__

// #include "foxglove-vn/Speed.pb.h"
#include "sim_data_flow/ST_msg.pb.h"
#include "svc/base_svc_serial.h"

#define SERIAL_MSG_BUF 128

namespace VNSim {
// TODO: 删除

class SVCMaster : public BaseSerialSvc {
    // TODO: 接口说明
   public:
    SVCMaster();
    ~SVCMaster();

   public:
    int onInitService();
    void subDownStreamCallBack(uint8_t *msg, int len);
    void pubUpStream();

   public:
    void onWebotMsg(const char *topic_name,
                    const eCAL::SReceiveCallbackData *data);

   private:
    struct Serial_Imu {
        double angle[3] = {0, 0, 0};
        double velocity[3] = {0, 0, 0};
        double acceleration[3] = {0, 0, 0};
    };

    struct Serial_ForkPose {
        // double x;
        // double y;
        double z = {0};
        // double raw;
        // double yaw;
        // double pitch;
    };

    struct WebotMsg {
        struct Serial_Imu imu;
        struct Serial_ForkPose forkPose;
        double last_wheel_position = 0;
        double wheel_yaw = 0;
    };

    struct ReportMsg {
        uint32_t dataidx = 0;
        uint32_t dataidx_upload = 0;
        double rpm = 0;
        int fork_state = 0;
        struct WebotMsg webot_msg;
    };
    struct ReportMsg report_msg_;
    bool rpm_init_;
    uint8_t buf[SERIAL_MSG_BUF];
    sim_data_flow::STMsg payload;
    sim_data_flow::STDown payload_Down;
};
}  // namespace VNSim

#endif