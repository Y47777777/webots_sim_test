
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
    uint32_t DataIndex = 0;
    double fakeDataIndex = 0;
    int16_t MoveDevice = 0;
    double realMoveDevice = 0;
    int16_t SteeringDevice = 0;
    double realSteeringDevice = 0;
    int16_t ForkDeviceZ = 0;
    double realForkDeviceZ = 0.02;
    double nothing = 0;
    uint8_t switch_sensor[10] = {0xFF};
    decoder_.decodePackage(&pack);
    decoder_.getValue("DataIndex", &DataIndex,
                      &fakeDataIndex);  // store in local
    decoder_.getValue("MoveDevice", &MoveDevice,
                      &realMoveDevice);  // steer wheel speed
    decoder_.getValue("SteeringDevice", &SteeringDevice,
                      &realSteeringDevice);  // steer yaw
    decoder_.getValue("ForkDeviceZ", &ForkDeviceZ,
                      &realForkDeviceZ);  // fork Speed
    // decoder_->getValue("SwitchActuator", &switch_sensor, &nothing); // in
    // std::cout << __FUNCTION__ << " DataIndex = " << DataIndex
    //           << ",MoveDevice = " << realMoveDevice
    //           << ",steeringDevice = " << realSteeringDevice
    //           << ",ForkDeviceZ = " << realForkDeviceZ << std::endl;
    // type2 ST it will be useful.
    payload_Down.set_forkspeedz(realForkDeviceZ);
    payload_Down.set_steering_speed(realMoveDevice);
    payload_Down.set_steering_theta(realSteeringDevice);
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_wrapper_.send("svc_model_st/ST_msg", buf, payload.ByteSize());
    report_msg_.dataidx = DataIndex;
    report_msg_.webot_msg.wheel_yaw = realSteeringDevice;
}

void SVCModelSerial::onUpStreamProcess() {
    uint8_t switchValue[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    std::vector<struct UpdateValue *> SwitchSencerInput;
    struct UpdateValue switchValueList[12];
    SwitchSencerInput.resize(12);

    uint16_t battery_device = 100;
    uint32_t d_upload = report_msg_.dataidx_upload++;
    struct UpdateValue battery_sencer_value = {&battery_device,
                                               sizeof(uint16_t)};
    struct UpdateValue dataIdx_value = {&report_msg_.dataidx, sizeof(uint32_t)};
    struct UpdateValue dataIdx_upload_value = {&d_upload, sizeof(uint32_t)};
    std::vector<double> incrementalCoder_cal_input;
    std::vector<double> GyroScope_cal_input;
    std::vector<double> AccelerometerXInput;
    std::vector<double> AccelerometerYInput;
    std::vector<double> AccelerometerZInput;
    std::vector<double> AngularVelocitySensorXInput;
    std::vector<double> AngularVelocitySensorYInput;
    std::vector<double> AngularVelocitySensorZInput;
    std::vector<double> RPMInput;
    std::vector<double> fake_cal_input;
    std::vector<struct UpdateValue *> fake_direct_input;
    std::vector<struct UpdateValue *> battery_sencer_input;
    std::vector<struct UpdateValue *> dataIdx_input;
    std::vector<struct UpdateValue *> dataIdx_upload_input;
    incrementalCoder_cal_input.push_back(
        report_msg_.webot_msg.wheel_yaw);  // steer_yaw
    GyroScope_cal_input.push_back(
        report_msg_.webot_msg.imu.angle[2]);  // vehicle_yaw
    RPMInput.push_back(report_msg_.rpm);
    AccelerometerXInput.push_back(report_msg_.webot_msg.imu.velocity[0]);  // X
    AccelerometerYInput.push_back(report_msg_.webot_msg.imu.velocity[1]);  // Y
    AccelerometerZInput.push_back(report_msg_.webot_msg.imu.velocity[2]);  // Z
    AngularVelocitySensorXInput.push_back(
        report_msg_.webot_msg.imu.acceleration[0]);  // roll
    AngularVelocitySensorYInput.push_back(
        report_msg_.webot_msg.imu.acceleration[1]);  // pitch
    AngularVelocitySensorZInput.push_back(
        report_msg_.webot_msg.imu.acceleration[2]);  // yaw
    battery_sencer_input.push_back(&battery_sencer_value);
    dataIdx_input.push_back(&dataIdx_value);
    dataIdx_upload_input.push_back(&dataIdx_upload_value);
    if (report_msg_.fork_state == int(FORK_STATE::ON_FORK_TOP)) {
        switchValue[4] = 0x01;
    } else if (report_msg_.fork_state == int(FORK_STATE::ON_FORK_BOTTOM)) {
        switchValue[4] = 0x02;
    } else {
        switchValue[4] = 0x00;
    }
    for (int i = 0; i < 12; i++) {
        switchValueList[i].val = &switchValue[i];
        switchValueList[i].len = 1;
        switchValueList[i].subId = i;
        SwitchSencerInput[i] = &switchValueList[i];
    }
    encoder_.updateValue("IncrementalSteeringCoder", incrementalCoder_cal_input,
                         fake_direct_input);
    encoder_.updateValue("Gyroscope", GyroScope_cal_input, fake_direct_input);
    encoder_.updateValue("RPMSensor", RPMInput, fake_direct_input);
    encoder_.updateValue("BatterySencer", fake_cal_input, battery_sencer_input);
    encoder_.updateValue("DataIndex", fake_cal_input, dataIdx_upload_input);
    encoder_.updateValue("DataIndexReturn", fake_cal_input, dataIdx_input);
    encoder_.updateValue("SwitchSencer", fake_cal_input, SwitchSencerInput);
    encoder_.updateValue("AccelerometerX", AccelerometerXInput,
                         fake_direct_input);
    encoder_.updateValue("AccelerometerY", AccelerometerYInput,
                         fake_direct_input);
    encoder_.updateValue("AccelerometerZ", AccelerometerZInput,
                         fake_direct_input);
    encoder_.updateValue("AngularVelocitySensorX", AngularVelocitySensorXInput,
                         fake_direct_input);  // roll
    encoder_.updateValue("AngularVelocitySensorY", AngularVelocitySensorYInput,
                         fake_direct_input);  // pitc
    encoder_.updateValue("AngularVelocitySensorZ", AngularVelocitySensorZInput,
                         fake_direct_input);  // yaw
    lidar_count_++;
    const struct Package *pack = encoder_.encodePackage();
    // std::cout << __FUNCTION__ << " RPM = " << RPMInput.at(0)
    //           << ", Gyroscope = " << GyroScope_cal_input.at(0)
    //           << ", IncrementalSteeringCoder = "
    //           << incrementalCoder_cal_input.at(0) << std::endl;
    ecal_wrapper_.send("Sensor/read", pack->buf, pack->len);
}