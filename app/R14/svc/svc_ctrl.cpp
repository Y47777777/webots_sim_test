#include "geometry/geometry.h"
#include "sim_data_flow/pose.pb.h"
#include "svc_ctrl.h"

using namespace VNSim;

std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;

SVCMaster::SVCMaster() : BaseSerialSvc() {}

SVCMaster::~SVCMaster() {}

int SVCMaster::onInitService() {
    // publisher
    ecal_ptr_->addEcal("Sensor/read");
    ecal_ptr_->addEcal("svc/R_msg");
    ecal_ptr_->addEcal("svc/pose");

    // Receive
    ecal_ptr_->addEcal("webot/R_msg",
                       std::bind(&SVCMaster::subRMsgCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&SVCMaster::subPoseCallBakc, this,
                                 std::placeholders::_1, std::placeholders::_2));
    return 0;
}

void SVCMaster::subRMsgCallBack(const char *topic_name,
                                const eCAL::SReceiveCallbackData *data) {
    // 反序列化
    msg_from_webots_.ParseFromArray(data->buf, data->size);

    // 上报
    pubUpStream();
}

void SVCMaster::subPoseCallBakc(const char *topic_name,
                                const eCAL::SReceiveCallbackData *data) {
    // 反序列化
    sim_data_flow::Pose pose;
    pose.ParseFromArray(data->buf, data->size);

    uint64_t time_stamp = pose.timestamp();
    pose.set_timestamp(Timer::getInstance()->getTimeFromBase(time_stamp));

    uint8_t buf[pose.ByteSize()];
    pose.SerializePartialToArray(buf, pose.ByteSize());
    ecal_ptr_->send("svc/pose", buf, pose.ByteSize());
}

void SVCMaster::subDownStreamCallBack(uint8_t *msg, int len) {
    struct Package pack {
        msg, len
    };

    // 解包到decoder_中
    decoder_.decodePackage(&pack);
    {
        // 该数据多线程读写
        std::lock_guard<std::mutex> lock(msgs_lock_);
        decoder_.getValue2("DataIndex", &dataidx_sub_, 4);  // 回报
    }

    pubRMsgsToWebots();
}

void SVCMaster::pubRMsgsToWebots() {
    double MoveDevice;
    double SteeringDevice;
    double ForkDeviceX;
    double ForkDeviceY;
    double ForkDeviceZ;
    double ForkDeviceP;
    bool   Lidar0MoveUp;
    bool   Lidar0MoveDown;

    decoder_.getValue("MoveDevice", &MoveDevice);          // steer wheel
    decoder_.getValue("SteeringDevice", &SteeringDevice);  // steer yaw
    decoder_.getValue("ForkDevice", &ForkDeviceX, "X");    // forkX Speed
    decoder_.getValue("ForkDevice", &ForkDeviceY, "Y");    // forkY Speed
    decoder_.getValue("ForkDevice", &ForkDeviceZ, "Z");    // forkZ Speed
    decoder_.getValue("ForkDevice", &ForkDeviceP, "P");    // forkP Speed
    decoder_.getSwitchValue("SwitchActuator", 48, &Lidar0MoveUp);  // lidar0感知举升
    decoder_.getSwitchValue("SwitchActuator", 49, &Lidar0MoveDown);  // lidar0感知下降

    msg_to_webots_.set_timestamp(Timer::getInstance()->getCurrentFromSystem());
    msg_to_webots_.set_steering_speed(MoveDevice);
    msg_to_webots_.set_steering_theta(SteeringDevice);
    msg_to_webots_.set_forkspeedx(ForkDeviceX);
    msg_to_webots_.set_forkspeedy(ForkDeviceY);
    msg_to_webots_.set_forkspeedz(ForkDeviceZ);
    msg_to_webots_.set_forkspeedp(ForkDeviceP);
    msg_to_webots_.set_lidar0_up(Lidar0MoveUp);
    msg_to_webots_.set_lidar0_down(Lidar0MoveDown);

    // publish
    uint8_t buf[msg_to_webots_.ByteSize()];
    msg_to_webots_.SerializePartialToArray(buf, msg_to_webots_.ByteSize());
    ecal_ptr_->send("svc/R_msg", buf, msg_to_webots_.ByteSize());
}

void SVCMaster::pubUpStream() {
    if (first_pub_report_) {
        // 第一帧发送为 系统 0时
        Timer::getInstance()->setBaseTime(msg_from_webots_.timestamp());
        first_pub_report_ = false;
        LOG_INFO("set base timer %d", Timer::getInstance()->getBaseTime());
    }
    foxglove::Imu *imu = msg_from_webots_.mutable_imu();
    // 数据转换
    encoder_.updateValue("IncrementalSteeringCoder", 1, "", msg_from_webots_.steering_theta());
    encoder_.updateValue("HeightCoder", 1, "", msg_from_webots_.forkposez());
    encoder_.updateValue("ForkDisplacementSencer", 1, "X", msg_from_webots_.forkposex()); //货叉前后移动
    encoder_.updateValue("ForkDisplacementSencer", 1, "Y", msg_from_webots_.forkposey()); //货叉左右横移
    encoder_.updateValue("Gyroscope", 1, "", msg_from_webots_.gyroscope());
    encoder_.updateValue2("DataIndex", &dataidx_upload_, sizeof(uint32_t));
    encoder_.updateValue("ElePerceptionCameraDistance", 1, "", msg_from_webots_.lidar0posez()); //感知位置

    uint16_t battery_device = 100;
    encoder_.updateValue2("BatterySencer", &battery_device, sizeof(uint16_t));

    //从动轮编码位置计算
    static double wheel_coder_l = 0;
    static double wheel_coder_r = 0;
    wheel_coder_l += msg_from_webots_.l_wheel() * 0.5;
    wheel_coder_r += msg_from_webots_.r_wheel() * 0.5;
    encoder_.updateValue("WheelCoder", 2, "", wheel_coder_l, wheel_coder_r);
    encoder_.updateSwitchValue("SwitchSencer", 8, msg_from_webots_.lforksafety()); //左货叉安全开关
    encoder_.updateSwitchValue("SwitchSencer", 9, msg_from_webots_.rforksafety()); //右货叉安全开关
    LOG_INFO("lf %s, rf %s", msg_from_webots_.lforksafety()? "on" : "off", msg_from_webots_.rforksafety()? "on" : "off");
    encoder_.updateSwitchValue("SwitchSencer", 50, msg_from_webots_.hswitchl()); //左横向到位开关 这里采用双压杆到位开关
    encoder_.updateSwitchValue("SwitchSencer", 51, msg_from_webots_.hswitchr()); //右横向到位开关
    encoder_.updateSwitchValue("SwitchSencer", 52, msg_from_webots_.vswitchl()); //左垂直到位开关
    encoder_.updateSwitchValue("SwitchSencer", 53, msg_from_webots_.vswitchr()); //右垂直到位开关
    encoder_.updateSwitchValue("SwitchSencer", 30, msg_from_webots_.is_lidar0orposez()); //感知是否在初始限位
    

    encoder_.updateValue("Accelerometer", 1, "X",
                         imu->linear_acceleration().x());
    encoder_.updateValue("Accelerometer", 1, "Y",
                         imu->linear_acceleration().y());
    encoder_.updateValue("Accelerometer", 1, "Z",
                         imu->linear_acceleration().z());
    encoder_.updateValue("AngularVelocitySensor", 1, "X",
                         imu->angular_velocity().x());
    encoder_.updateValue("AngularVelocitySensor", 1, "Y",
                         imu->angular_velocity().y());
    encoder_.updateValue("AngularVelocitySensor", 1, "Z",
                         imu->angular_velocity().z());
    {
        // 该数据多线程读写
        std::lock_guard<std::mutex> lock(msgs_lock_);
        encoder_.updateValue2("DataIndexReturn", &dataidx_sub_,
                              sizeof(uint32_t));
    }

    const struct Package *pack = encoder_.encodePackage();
    ecal_ptr_->send("Sensor/read", pack->buf, pack->len);

    dataidx_upload_++;
}