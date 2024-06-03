#ifndef __SVC_MODEL_SERIAL_H__
#define __SVC_MODEL_SERIAL_H__

#include "foxglove-vn/Speed.pb.h"
#include "sim_data_flow/ST_msg.pb.h"
#include "base_svc_serial.h"

#define SERIAL_MSG_BUF 128

namespace VNSim {
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

class SVCModelSerial : public BaseSerialSVCModel {
   private:
    struct ReportMsg report_msg_;
    bool rpm_init_;
    uint8_t buf[SERIAL_MSG_BUF];
    sim_data_flow::STMsg payload;
    sim_data_flow::STDown payload_Down;

   public:
    SVCModelSerial();
    ~SVCModelSerial();

   public:
    int onInitService();
    void onDownStreamProcess(uint8_t *msg, int len);
    void onUpStreamProcess();

   public:
    void onWebotMsg(const char *topic_name,
                    const eCAL::SReceiveCallbackData *data);
};
}  // namespace VNSim

#endif