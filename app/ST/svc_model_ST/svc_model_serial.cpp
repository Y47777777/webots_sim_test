
#include "svc_model_serial.h"

// #define SERIAL_MSG_BUF 128
#define SIM_FAC 1.06695119
#define FORK_LIFTUP_HEIGHT 0.085
#define FORK_LIFTDOWN_HEIGHT 0
#define HEIGHT_DEVIATION 0.0001

enum FORK_STATE { ON_FORK_BOTTOM = 0, ON_FORK_MIDDLE = 1, ON_FORK_TOP = 2 };

using namespace VNSim;

SVCModelSerial::SVCModelSerial() : BaseSerialSVCModel(), rpm_init_(false) {}

SVCModelSerial::~SVCModelSerial() {}

int SVCModelSerial::onInitService() {
    // send msg to general
    ecal_wrapper_.addEcal("Sensor/read");
    // Receive
    ecal_wrapper_.addEcal("webot/ST_msg", std::bind(&SVCModelSerial::onWebotMsg,
                                                    this, std::placeholders::_1,
                                                    std::placeholders::_2));
    ecal_wrapper_.addEcal("svc_model_st/ST_msg");
    payload.set_allocated_down_msg(&payload_Down);

    return 0;
}

void SVCModelSerial::onWebotMsg(const char *topic_name,
                                const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::STMsg payload;
    payload.ParseFromArray(data->buf, data->size);
    report_msg_.webot_msg.imu.angle[2] =
        payload.up_msg().imu().orientation_covariance(0);  // vehicle_yaw
    double forkZ = payload.up_msg().forkposez();           // forkZ Height
    if (std::abs(forkZ - FORK_LIFTUP_HEIGHT) <= HEIGHT_DEVIATION) {
        report_msg_.fork_state = int(FORK_STATE::ON_FORK_TOP);
    } else if (std::abs(forkZ - FORK_LIFTDOWN_HEIGHT) <= HEIGHT_DEVIATION) {
        report_msg_.fork_state = int(FORK_STATE::ON_FORK_BOTTOM);
    } else {
        report_msg_.fork_state = int(FORK_STATE::ON_FORK_MIDDLE);
    }
    for (int i = 0; i < 3; i++) {
        report_msg_.webot_msg.imu.velocity[i] =
            payload.up_msg().imu().angular_velocity_covariance(
                i);  // angular_velocity
        report_msg_.webot_msg.imu.acceleration[i] =
            payload.up_msg().imu().linear_acceleration_covariance(
                i);  // accelerator
    }
    double wheel_position = payload.up_msg().steerposition();
    if (!rpm_init_) {
        report_msg_.webot_msg.last_wheel_position = wheel_position;
        rpm_init_ = true;
    }
    report_msg_.rpm =
        (wheel_position - report_msg_.webot_msg.last_wheel_position) * SIM_FAC;
    report_msg_.webot_msg.last_wheel_position = wheel_position;
}

void SVCModelSerial::onDownStreamProcess(uint8_t *msg, int len) {
    struct Package pack {
        msg, len
    };
    double data_idx;
    double ForkDeviceZ;
    double SteeringDevice;
    double MoveDevice;
    decoder_.decodePackage(&pack);
    decoder_.getValue2("DataIndex", &data_idx, 4);         // store in local
    decoder_.getValue("MoveDevice", &MoveDevice);          // steer wheel speed
    decoder_.getValue("SteeringDevice", &SteeringDevice);  // steer yaw
    decoder_.getValue("ForkDeviceZ", &ForkDeviceZ);        // fork Speed
    // type2 ST it will be useful.
    payload_Down.set_forkspeedz(ForkDeviceZ);
    payload_Down.set_steering_speed(MoveDevice);
    payload_Down.set_steering_theta(SteeringDevice);
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_wrapper_.send("svc_model_st/ST_msg", buf, payload.ByteSize());
    report_msg_.dataidx = data_idx;
    report_msg_.webot_msg.wheel_yaw = SteeringDevice;
}

void SVCModelSerial::onUpStreamProcess() {
    uint16_t battery_device = 100;
    uint32_t dataidx_upload = report_msg_.dataidx_upload++;
    bool fork[2] = {false, false};
    const char Axis[3] = {'X', 'Y', 'Z'};
    // 38 is up, 39 is down
    if (report_msg_.fork_state == int(FORK_STATE::ON_FORK_TOP)) {
        fork[0] = true;
    } else if (report_msg_.fork_state == int(FORK_STATE::ON_FORK_BOTTOM)) {
        fork[1] = true;
    }
    // Data to Upload
    std::string Imu_Function = "";
    encoder_.updateValue("IncrementalSteeringCoder", 1,
                         report_msg_.webot_msg.wheel_yaw);
    encoder_.updateValue("Gyroscope", 1, report_msg_.webot_msg.imu.angle[2]);
    encoder_.updateValue("RPMSensor", 1, report_msg_.rpm);
    encoder_.updateValue2("BatterySencer", &battery_device, sizeof(uint16_t));
    encoder_.updateValue2("DataIndex", &dataidx_upload, sizeof(uint32_t));
    encoder_.updateValue2("DataIndexReturn", &report_msg_.dataidx,
                          sizeof(uint32_t));
    for (int i = 0; i < 2; i++)
        encoder_.updateSwitchValue("SwitchSencer", 38 + i, fork[i]);
    for (int i = 0; i < 3; i++) {
        Imu_Function = "Accelerometer" + std::string(&Axis[i]);
        encoder_.updateValue(Imu_Function.c_str(), 1,
                             report_msg_.webot_msg.imu.acceleration[i]);
        Imu_Function = "AngularVelocitySensor" + std::string(&Axis[i]);
        encoder_.updateValue(Imu_Function.c_str(), 1,
                             report_msg_.webot_msg.imu.velocity[i]);
    }
    // lidar_count_++;
    const struct Package *pack = encoder_.encodePackage();
    ecal_wrapper_.send("Sensor/read", pack->buf, pack->len);
}