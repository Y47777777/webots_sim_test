#include "geometry/geometry.h"
#include "svc_model_serial.h"

// #define SERIAL_MSG_BUF 128
#define SIM_FAC 1.06695119
#define FORK_LIFTUP_HEIGHT 0.085
#define FORK_LIFTDOWN_HEIGHT 0
#define HEIGHT_DEVIATION 0.0001
#define SPEED_REAL_WORLD_TO_SIMULATION 7.8863

enum FORK_STATE { ON_FORK_BOTTOM = 0, ON_FORK_MIDDLE = 1, ON_FORK_TOP = 2 };

using namespace VNSim;

std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;

SVCModelSerial::SVCModelSerial() : BaseSerialSVCModel(), rpm_init_(false) {}

SVCModelSerial::~SVCModelSerial() {}

int SVCModelSerial::onInitService() {
    // send msg to general
    ecal_ptr_->addEcal("Sensor/read");
    // Receive
    ecal_ptr_->addEcal("webot/P_msg",
                       std::bind(&SVCModelSerial::onWebotMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));
    ecal_ptr_->addEcal("svc_model_st/P_msg");
    payload.set_allocated_down_msg(&payload_Down);
    return 0;
}

void SVCModelSerial::onWebotMsg(const char *topic_name,
                                const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::PUp payload2;
    payload2.ParseFromArray(data->buf, data->size);
    foxglove::Imu *imu = payload2.mutable_imu();
    // use a spin lock to manage data copy
    // if the time consumption is huge, use a real mutex instead
    {
        std::shared_lock<std::shared_mutex> lock(lock_mutex_);
        // This period should be locked
        Eigen::Quaterniond t(
            imu->mutable_orientation()->x(), imu->mutable_orientation()->y(),
            imu->mutable_orientation()->z(), imu->mutable_orientation()->w());
        report_msg_.webot_msg.imu.angle[2] = imu->mutable_orientation()->w();
        // FIXME: current imu vehicle yaw is not correct, use old function
        // P车编码器为绝对式
        // report_msg_.webot_msg.l_wheel += payload2.l_wheel() / 6.28 * 1000;
        // report_msg_.webot_msg.r_wheel += payload2.r_wheel() / 6.28 * 1000;
        report_msg_.webot_msg.l_wheel +=
            (payload2.l_wheel() * 0.105);  // give arc length
        report_msg_.webot_msg.r_wheel +=
            (payload2.r_wheel() * 0.105);  // give arc length
        report_msg_.webot_msg.forkPose.z = payload2.forkposez();
        report_msg_.webot_msg.imu.velocity[0] = imu->angular_velocity().x();
        report_msg_.webot_msg.imu.velocity[1] = imu->angular_velocity().y();
        report_msg_.webot_msg.imu.velocity[2] = imu->angular_velocity().z();
        report_msg_.webot_msg.imu.acceleration[0] =
            imu->linear_acceleration().x();
        report_msg_.webot_msg.imu.acceleration[1] =
            imu->linear_acceleration().y();
        report_msg_.webot_msg.imu.acceleration[2] =
            imu->linear_acceleration().z();
        double wheel_position = payload2.steerposition();
        if (!rpm_init_) {
            report_msg_.webot_msg.last_wheel_position = wheel_position;
            rpm_init_ = true;
        }
        report_msg_.rpm =
            (wheel_position - report_msg_.webot_msg.last_wheel_position) *
            SIM_FAC;
        report_msg_.webot_msg.last_wheel_position = wheel_position;
    }
}

void SVCModelSerial::onDownStreamProcess(uint8_t *msg, int len) {
    struct Package pack {
        msg, len
    };
    uint32_t data_idx;
    double ForkDeviceZ;
    double SteeringDevice;
    double MoveDevice;
    decoder_.decodePackage(&pack);
    decoder_.getValue2("DataIndex", &data_idx, 4);         // store in local
    decoder_.getValue("MoveDevice", &MoveDevice);          // steer wheel
    decoder_.getValue("SteeringDevice", &SteeringDevice);  // steer yaw
    decoder_.getValue("ForkDeviceZ", &ForkDeviceZ);        // fork Speed
    // type2 ST it will be useful.
    payload_Down.set_forkspeedz(ForkDeviceZ);
    // payload_Down.set_steering_speed(MoveDevice * 2.53807107);
    payload_Down.set_steering_speed(MoveDevice / 0.4);
    payload_Down.set_steering_theta(SteeringDevice);

    // publish
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("svc_model_st/P_msg", buf, payload.ByteSize());
    {
        // This period should be locked
        std::shared_lock<std::shared_mutex> lock(lock_mutex_);
        report_msg_.dataidx = data_idx;
        // TODO: ？？发布数据直接写入回复？
        report_msg_.webot_msg.wheel_yaw = SteeringDevice;
    }
}

void SVCModelSerial::onUpStreamProcess() {
    uint16_t battery_device = 100;
    uint32_t dataidx_upload = report_msg_.dataidx_upload++;
    bool fork[2] = {false, false};
    const char Axis[3] = {'X', 'Y', 'Z'};
    uint32_t l_dataidx = 0;
    double l_steer_yaw = 0;
    // double l_rpm = 0;
    double l_l = 0;
    double l_r = 0;
    double l_forkZ = 0;
    struct Serial_Imu l_Imu;
    {
        // This period should be locked
        std::unique_lock<std::shared_mutex> lock(lock_mutex_);
        l_dataidx = report_msg_.dataidx;
        l_steer_yaw = report_msg_.webot_msg.wheel_yaw;
        l_l = report_msg_.webot_msg.l_wheel;
        l_r = report_msg_.webot_msg.r_wheel;
        // l_rpm = report_msg_.rpm;
        // 38 is down, 39 is up
        // if (report_msg_.fork_state == int(FORK_STATE::ON_FORK_TOP)) {
        //     fork[1] = true;
        // } else if (report_msg_.fork_state == int(FORK_STATE::ON_FORK_BOTTOM))
        // {
        //     fork[0] = true;
        // }
        l_forkZ = report_msg_.webot_msg.forkPose.z;
        l_Imu.angle[2] = report_msg_.webot_msg.imu.angle[2];

        for (int i = 0; i < 3; i++) {
            l_Imu.acceleration[i] = report_msg_.webot_msg.imu.acceleration[i];
            l_Imu.velocity[i] = report_msg_.webot_msg.imu.velocity[i];
        }
    }

    // Data to Upload
    std::string Imu_Function = "";
    encoder_.updateValue("IncrementalSteeringCoder", 1, l_steer_yaw);
    encoder_.updateValue("Gyroscope", 1, l_Imu.angle[2]);
    // encoder_.updateValue("RPMSensor", 1, l_rpm);
    encoder_.updateValue2("BatterySencer", &battery_device, sizeof(uint16_t));
    encoder_.updateValue2("DataIndex", &dataidx_upload, sizeof(uint32_t));
    encoder_.updateValue2("DataIndexReturn", &l_dataidx, sizeof(uint32_t));
    encoder_.updateValue("WheelCoder", 2, l_l, l_r);
    encoder_.updateValue("HeightCoder", 1, l_forkZ);
    encoder_.updateValue("ForkDisplacementSencerZ", 1, l_forkZ);
    // for (int i = 0; i < 2; i++)
    //     encoder_.updateSwitchValue("SwitchSencer", 38 + i, fork[i]);
    // for (int i = 0; i < 3; i++) {
    //     Imu_Function = "Accelerometer" + std::string(&Axis[i]);
    //     encoder_.updateValue(Imu_Function.c_str(), 1, l_Imu.acceleration[i]);
    //     Imu_Function = "AngularVelocitySensor" + std::string(&Axis[i]);
    //     encoder_.updateValue(Imu_Function.c_str(), 1, l_Imu.velocity[i]);
    // }
    // std::cout << "l_steer_yaw = " << l_steer_yaw
    //           << ", Gyroscope = " << l_Imu.angle[2] << ", WheelCoder = [" <<
    //           l_l
    //           << "," << l_r << "] , forkZ = " << l_forkZ << std::endl;
    std::cout << "report yaw = " << l_Imu.angle[2] << std::endl;
    const struct Package *pack = encoder_.encodePackage();
    ecal_ptr_->send("Sensor/read", pack->buf, pack->len);
}