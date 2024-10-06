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
    ecal_ptr_->addEcal("svc/ST_msg");
    ecal_ptr_->addEcal("svc/pose");

    // Receive
    ecal_ptr_->addEcal("webot/ST_msg",
                       std::bind(&SVCMaster::subSTMsgCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&SVCMaster::subPoseCallBakc, this,
                                 std::placeholders::_1, std::placeholders::_2));
    return 0;
}

void SVCMaster::subSTMsgCallBack(const char *topic_name,
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

    pubSTMsgsToWebots();
}

void SVCMaster::pubSTMsgsToWebots() {
    double ForkDeviceZ;
    double SteeringDevice;
    double MoveDevice;

    decoder_.getValue("MoveDevice", &MoveDevice);          // steer wheel
    decoder_.getValue("SteeringDevice", &SteeringDevice);  // steer yaw
    decoder_.getValue("ForkDevice", &ForkDeviceZ, "Z");    // fork Speed

    msg_to_webots_.set_steering_speed(MoveDevice);
    msg_to_webots_.set_steering_theta(SteeringDevice);
    msg_to_webots_.set_forkspeedz(ForkDeviceZ);

    // publish
    uint8_t buf[msg_to_webots_.ByteSize()];
    msg_to_webots_.SerializePartialToArray(buf, msg_to_webots_.ByteSize());
    ecal_ptr_->send("svc/ST_msg", buf, msg_to_webots_.ByteSize());
}

void SVCMaster::pubUpStream() {
    if (first_pub_report_) {
        // 第一帧发送为 系统 0时
        Timer::getInstance()->setBaseTime(msg_from_webots_.timestamp());
        first_pub_report_ = false;
        LOG_INFO("set base timer %d", Timer::getInstance()->getBaseTime());
    }

    // 数据转换
    encoder_.updateValue("IncrementalSteeringCoder", 1, "",msg_from_webots_.steering_theta());
    encoder_.updateValue("Gyroscope",                1, "", msg_from_webots_.gyroscope());
    encoder_.updateValue("HeightCoder",              1, "", msg_from_webots_.forkposez());
    encoder_.updateValue("ForkDisplacementSencer",   1, "Z",msg_from_webots_.forkposez());
    encoder_.updateValue("RPMSensor",                1, "", msg_from_webots_.steerposition());
    encoder_.updateValue2("DataIndex", &dataidx_upload_, sizeof(uint32_t));

    foxglove::Imu *imu = msg_from_webots_.mutable_imu();
    encoder_.updateValue("AngularVelocitySensor", 1, "X", imu->angular_velocity().x());
    encoder_.updateValue("AngularVelocitySensor", 1, "Y", imu->angular_velocity().y());
    encoder_.updateValue("AngularVelocitySensor", 1, "Z", imu->angular_velocity().z());
    encoder_.updateValue("Accelerometer",         1, "X", imu->linear_acceleration().x());
    encoder_.updateValue("Accelerometer",         1, "Y",imu->linear_acceleration().y());
    encoder_.updateValue("Accelerometer",         1, "Z",imu->linear_acceleration().z());

    // encoder_.updateSwitchValue("SwitchSencer", 38 + i, fork[i]);

    uint16_t battery_device = 100;
    encoder_.updateValue2("BatterySencer", &battery_device, sizeof(uint16_t));
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