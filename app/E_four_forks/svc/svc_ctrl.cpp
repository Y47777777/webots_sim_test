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
    ecal_ptr_->addEcal("svc/E_msg");
    ecal_ptr_->addEcal("svc/pose");

    // Receive
    ecal_ptr_->addEcal("webot/E_msg",
                       std::bind(&SVCMaster::subEMsgCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&SVCMaster::subPoseCallBakc, this,
                                 std::placeholders::_1, std::placeholders::_2));
    return 0;
}

void SVCMaster::subEMsgCallBack(const char *topic_name,
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

    pubEMsgsToWebots();
}

void SVCMaster::pubEMsgsToWebots() {
    double ForkDeviceZ;
    double ForkDeviceY;
    double ForkDeviceP;
    double ForkDeviceC;
    double SteeringDeviceL;
    double SteeringDeviceR;
    double MoveDevice;
    bool isPowerOff = false;

    decoder_.getValue("MoveDevice", &MoveDevice);  // steer wheel
    decoder_.getValue("SteeringDevice", &SteeringDeviceL,
                      "LF");  // steer yaw LF
    decoder_.getValue("SteeringDevice", &SteeringDeviceR,
                      "RF");                             // steer yaw RF
    decoder_.getValue("ForkDevice", &ForkDeviceZ, "Z");  // fork Speed
    decoder_.getValue("ForkDevice", &ForkDeviceY, "Y");
    decoder_.getValue("ForkDevice", &ForkDeviceP, "P");
    decoder_.getValue("ForkDevice", &ForkDeviceC, "C");
    decoder_.getSwitchValue("SwitchActuator",0, &isPowerOff);
    if(isPowerOff){
        LOG_WARN("%s --> Poweroff read value = %d", __FUNCTION__, isPowerOff);
    }

    msg_to_webots_.set_steering_speed(MoveDevice);
    msg_to_webots_.set_steering_theta_l(SteeringDeviceL);
    msg_to_webots_.set_steering_theta_r(SteeringDeviceR);
    msg_to_webots_.set_forkspeedz(ForkDeviceZ);
    msg_to_webots_.set_forkspeedy(ForkDeviceY);
    msg_to_webots_.set_forkspeedp(ForkDeviceP);
    msg_to_webots_.set_forkspeedc(ForkDeviceC);
    msg_to_webots_.set_ispoweroff(isPowerOff);

    // publish
    uint8_t buf[msg_to_webots_.ByteSize()];
    msg_to_webots_.SerializePartialToArray(buf, msg_to_webots_.ByteSize());
    ecal_ptr_->send("svc/E_msg", buf, msg_to_webots_.ByteSize());
}

void SVCMaster::pubUpStream() {
    if (first_pub_report_) {
        // 第一帧发送为 系统 0时
        Timer::getInstance()->setBaseTime(msg_from_webots_.timestamp());
        first_pub_report_ = false;
        LOG_INFO("set base timer %d", Timer::getInstance()->getBaseTime());
    }
    const char Axis[3] = {'X', 'Y', 'Z'};
    foxglove::Imu *imu = msg_from_webots_.mutable_imu();
    // 数据转换
    encoder_.updateValue("IncrementalSteeringCoder", 1, "LF",
                         msg_from_webots_.steering_theta_l());
    encoder_.updateValue("IncrementalSteeringCoder", 1, "RF",
                         msg_from_webots_.steering_theta_r());
    encoder_.updateValue("Gyroscope", 1, "", msg_from_webots_.gyroscope().z());
    encoder_.updateValue("Gyroscope", 1, "X", msg_from_webots_.gyroscope().x());
    encoder_.updateValue("Gyroscope", 1, "Y", msg_from_webots_.gyroscope().y());

    encoder_.updateValue("ForkDisplacementSencer", 1, "RC",
                         msg_from_webots_.forkposecr());
    encoder_.updateValue("ForkDisplacementSencer", 1, "LC",
                         msg_from_webots_.forkposecl());
    encoder_.updateValue("ForkDisplacementSencer", 1, "Y",
                         msg_from_webots_.forkposey());
    encoder_.updateValue("ForkDisplacementSencer", 1, "P",
                         msg_from_webots_.forkposep());
    encoder_.updateValue("HeightCoder", 1, "", msg_from_webots_.forkposez());
    encoder_.updateValue2("DataIndex", &dataidx_upload_, sizeof(uint32_t));

    uint16_t battery_device = 100;
    encoder_.updateValue2("BatterySencer", &battery_device, sizeof(uint16_t));

    static double wheel_coder_l = 0;
    static double wheel_coder_r = 0;
    wheel_coder_l += msg_from_webots_.l_wheel() * 0.5;
    wheel_coder_r += msg_from_webots_.r_wheel() * 0.5;
    // LOG_INFO("l_wheel_msg = %lf r_wheel_msg = %lf, wheel_l = %lf, wheel_r =
    // %lf\n", msg_from_webots_.l_wheel(), msg_from_webots_.r_wheel(),
    // wheel_coder_l, wheel_coder_r);
    encoder_.updateValue("WheelCoder", 2, "", wheel_coder_l, wheel_coder_r);
    encoder_.updateValue("AngularVelocitySensor", 1, "X",
                         imu->angular_velocity().x());
    encoder_.updateValue("AngularVelocitySensor", 1, "Y",
                         imu->angular_velocity().y());
    encoder_.updateValue("AngularVelocitySensor", 1, "Z",
                         imu->angular_velocity().z());
    encoder_.updateValue("Accelerometer", 1, "X",
                         imu->linear_acceleration().x());
    encoder_.updateValue("Accelerometer", 1, "Y",
                         imu->linear_acceleration().y());
    encoder_.updateValue("Accelerometer", 1, "Z",
                         imu->linear_acceleration().z());
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

void SVCMaster::onWatchDogHungry(){
    msg_to_webots_.set_ispoweroff(true);
    // publish
    uint8_t buf[msg_to_webots_.ByteSize()];
    msg_to_webots_.SerializePartialToArray(buf, msg_to_webots_.ByteSize());
    LOG_INFO("%s --> svc/E_msg ispoweroff set true", __FUNCTION__);
    printf("%s --> svc/E_msg ispoweroff set true\n", __FUNCTION__);
    ecal_ptr_->send("svc/E_msg", buf, msg_to_webots_.ByteSize());
}
