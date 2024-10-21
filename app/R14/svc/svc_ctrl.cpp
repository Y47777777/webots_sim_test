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

    decoder_.getValue("MoveDevice", &MoveDevice);          // steer wheel
    decoder_.getValue("SteeringDevice", &SteeringDevice);  // steer yaw
    decoder_.getValue("ForkDevice", &ForkDeviceX, "X");    // forkX Speed
    decoder_.getValue("ForkDevice", &ForkDeviceY, "Y");    // forkY Speed
    decoder_.getValue("ForkDevice", &ForkDeviceZ, "Z");    // forkZ Speed
    decoder_.getValue("ForkDevice", &ForkDeviceP, "P");    // forkP Speed

    msg_to_webots_.set_timestamp(Timer::getInstance()->getCurrentFromSystem());
    msg_to_webots_.set_steering_speed(MoveDevice);
    msg_to_webots_.set_steering_theta(SteeringDevice);
    msg_to_webots_.set_forkspeedx(ForkDeviceX);
    msg_to_webots_.set_forkspeedy(ForkDeviceY);
    msg_to_webots_.set_forkspeedz(ForkDeviceZ);
    msg_to_webots_.set_forkspeedp(ForkDeviceP);

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
    encoder_.updateValue("Gyroscope", 1, "", msg_from_webots_.gyroscope());
    encoder_.updateValue("ForkDisplacementSencer", 1, "Y", msg_from_webots_.forkposey()); //货叉横移
    encoder_.updateValue("DisplacementSencer", 1, "X", msg_from_webots_.forkposex()); //货叉左右移动
    encoder_.updateValue("ForkDisplacementSencer", 1, "Z", msg_from_webots_.forkposez());
    encoder_.updateValue("DisplacementSencer", 1, "P", msg_from_webots_.forkposep()); //货叉俯仰角度
    encoder_.updateValue2("DataIndex", &dataidx_upload_, sizeof(uint32_t));

    uint16_t battery_device = 100;
    encoder_.updateValue2("BatterySencer", &battery_device, sizeof(uint16_t));

    //从动轮编码位置计算
    static double wheel_coder_l = 0;
    static double wheel_coder_r = 0;
    wheel_coder_l += msg_from_webots_.l_wheel() * 0.5;
    wheel_coder_r += msg_from_webots_.r_wheel() * 0.5;
    encoder_.updateValue("WheelCoder", 2, "", wheel_coder_l, wheel_coder_r);
    encoder_.updateSwitchValue("SwitchSencer", 36, msg_from_webots_.hswitchl()); //左横向到位开关 这里采用双压杆到位开关
    encoder_.updateSwitchValue("SwitchSencer", 37, msg_from_webots_.hswitchr()); //右横向到位开关
    encoder_.updateSwitchValue("SwitchSencer", 38, msg_from_webots_.vswitchl()); //左纵向到位开关
    encoder_.updateSwitchValue("SwitchSencer", 39, msg_from_webots_.vswitchr()); //右纵向到位开关
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