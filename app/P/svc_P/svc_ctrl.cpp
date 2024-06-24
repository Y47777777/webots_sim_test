#include "geometry/geometry.h"
#include "svc_ctrl.h"

using namespace VNSim;

std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;

SVCMaster::SVCMaster() : BaseSerialSvc() {}

SVCMaster::~SVCMaster() {}

int SVCMaster::onInitService() {
    // send msg to general
    ecal_ptr_->addEcal("Sensor/read");
    ecal_ptr_->addEcal("svc/P_msg");

    // Receive
    ecal_ptr_->addEcal("webot/P_msg",
                       std::bind(&SVCMaster::subPMsgCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

void SVCMaster::subPMsgCallBack(const char *topic_name,
                                     const eCAL::SReceiveCallbackData *data) {
    // 反序列化
    msg_from_webots_.ParseFromArray(data->buf, data->size);

    // 上报
    pubUpStream();
}

void SVCMaster::subDownStreamCallBack(uint8_t *msg, int len) {
    struct Package pack {
        msg, len
    };

    // 解包到decoder_中
    decoder_.decodePackage(&pack);

    pubPMsgsToWebots();
}

void SVCMaster::pubPMsgsToWebots(){
    double ForkDeviceZ;
    double SteeringDevice;
    double MoveDevice;

    {
        // 该数据多线程读写
        std::lock_guard<std::mutex> lock(msgs_lock_);
        decoder_.getValue2("DataIndex",     &dataidx_sub_, 4);      // 回报
    }

    decoder_.getValue("MoveDevice",     &MoveDevice);           // steer wheel
    decoder_.getValue("SteeringDevice", &SteeringDevice);       // steer yaw
    decoder_.getValue("ForkDeviceZ",    &ForkDeviceZ);          // fork Speed

    msg_to_webots_.set_steering_speed   (MoveDevice);
    msg_to_webots_.set_steering_theta   (SteeringDevice);
    msg_to_webots_.set_forkspeedz       (ForkDeviceZ);

    // publish
    uint8_t buf[msg_to_webots_.ByteSize()];
    msg_to_webots_.SerializePartialToArray(buf, msg_to_webots_.ByteSize());
    ecal_ptr_->send("svc/P_msg", buf, msg_to_webots_.ByteSize());
}

void SVCMaster::pubUpStream() {
    if (first_pub_report_) {
        // 第一帧发送为 系统 0时
        Timer::getInstance()->setBaseTime(msg_from_webots_.timestamp());
        first_pub_report_ = false;
        LOG_INFO("set base timer %d", Timer::getInstance()->getBaseTime());
    }

    // 数据转换
    encoder_.updateValue ("IncrementalSteeringCoder",   1, msg_from_webots_.steering_theta());
    encoder_.updateValue ("Gyroscope",                  1, msg_from_webots_.gyroscope());
    encoder_.updateValue ("HeightCoder",                1, msg_from_webots_.forkposez());
    encoder_.updateValue ("ForkDisplacementSencerZ",    1, msg_from_webots_.forkposez()); //TODO: ?两个一样的？
    encoder_.updateValue2("DataIndex",                  &dataidx_upload_,  sizeof(uint32_t));

    uint16_t battery_device = 100;
    encoder_.updateValue2("BatterySencer",      &battery_device,   sizeof(uint16_t));

    double wheel_coder_l = 0;
    double wheel_coder_r = 0;
    wheel_coder_l += msg_from_webots_.l_wheel()* 0.5;
    wheel_coder_r += msg_from_webots_.r_wheel()* 0.5;
    encoder_.updateValue ("WheelCoder",          2, wheel_coder_l, wheel_coder_r);

    
    {
        // 该数据多线程读写
        std::lock_guard<std::mutex> lock(msgs_lock_);
        encoder_.updateValue2("DataIndexReturn",            &dataidx_sub_,     sizeof(uint32_t));
    }

    const struct Package *pack = encoder_.encodePackage();
    ecal_ptr_->send("Sensor/read", pack->buf, pack->len);

    dataidx_upload_++;
}