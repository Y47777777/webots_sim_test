/**
 * @file svc_sensor.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-06-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <shared_mutex>
#include "dataTransform.h"
#include "svc_sensor.h"
#include "sim_data_flow/point_cloud2.pb.h"
#include "sim_data_flow/point_cloud.pb.h"

using namespace VNSim;

std::string slam_1_webots_topic = "webots/Lidar/.54/PointCloud";
std::string slam_2_webots_topic = "webots/Lidar/.56/PointCloud";
std::string slam_3_webots_topic = "webots/Lidar/.57/PointCloud";
std::string perception_webots_topic = "webots/Lidar/.100/PointCloud";
std::string be_webots_topic = "webots/Lidar/.200/PointCloud";

std::string slam_1_webots_base_topic = "webots/LidarToBase/.54/PointCloud";
std::string slam_2_webots_base_topic = "webots/LidarToBase/.56/PointCloud";
std::string slam_3_webots_base_topic = "webots/LidarToBase/.57/PointCloud";

std::string slam_1_real_topic = "192.168.1.54";
std::string slam_2_real_topic = "192.168.1.56";
std::string slam_3_real_topic = "192.168.1.57";
std::string perception_real_topic = "192.168.1.100";
std::string be_real_topic = "192.168.1.200";

std::string multi_mid360_topic = "multi_mid360";

SVCShadow::SVCShadow() : BaseSvc() {}

SVCShadow::~SVCShadow() {}

int SVCShadow::initService() {
    // pub
    ecal_ptr_->addEcal(slam_1_real_topic.c_str());
    ecal_ptr_->addEcal(slam_2_real_topic.c_str());
    ecal_ptr_->addEcal(slam_3_real_topic.c_str());
    ecal_ptr_->addEcal(perception_real_topic.c_str());
    ecal_ptr_->addEcal(be_real_topic.c_str());
    ecal_ptr_->addEcal(multi_mid360_topic.c_str());

    // sub
    // TODO: 可以用匿名函数套一手
    ecal_ptr_->addEcal(slam_1_webots_topic.c_str(),
                       std::bind(&SVCShadow::onSlam1Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));
    ecal_ptr_->addEcal(slam_2_webots_topic.c_str(),
                       std::bind(&SVCShadow::onSlam2Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(slam_3_webots_topic.c_str(),
                       std::bind(&SVCShadow::onSlam3Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(perception_webots_topic.c_str(),
                       std::bind(&SVCShadow::onPerceptionMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(be_webots_topic.c_str(),
                       std::bind(&SVCShadow::onBrighteyeMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(slam_1_webots_base_topic.c_str(),
                       std::bind(&SVCShadow::onMultiMid360Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(slam_2_webots_base_topic.c_str(),
                       std::bind(&SVCShadow::onMultiMid360Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(slam_3_webots_base_topic.c_str(),
                       std::bind(&SVCShadow::onMultiMid360Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    return 0;
}

// TODO: 是不是可以归一成一条函数(如何避免多线程？)
void SVCShadow::onSlam1Msg(const char *topic_name,
                           const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;
    payload.ParseFromArray(data->buf, data->size);
    pbTopb2(payload, payload_send, seq++, 2);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(slam_1_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onSlam2Msg(const char *topic_name,
                           const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq++, 3);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(slam_2_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onSlam3Msg(const char *topic_name,
                           const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;

    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq++, 4);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(slam_3_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onPerceptionMsg(const char *topic_name,
                                const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;

    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq++, 0);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(perception_real_topic.c_str(), buf,
                    payload_send.ByteSize());
}

void SVCShadow::onBrighteyeMsg(const char *topic_name,
                               const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;

    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(be_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onMultiMid360Msg(const char *topic_name,
                                 const eCAL::SReceiveCallbackData *data) {
    // 从回调读出                                    
    sim_data_flow::WBPointCloud payload;
    static uint64_t seq = 0;
    payload.ParseFromArray(data->buf, data->size);

    // 拷贝至缓存
    static std::shared_mutex rw_mutex_;  // 读写锁
    static bool slam1_recive_ = false;
    static bool slam2_recive_ = false;
    static bool slam3_recive_ = false;
    static sim_data_flow::WBPointCloud payload_slam1_;
    static sim_data_flow::WBPointCloud payload_slam2_;
    static sim_data_flow::WBPointCloud payload_slam3_;

    if (strcmp(topic_name, slam_1_webots_base_topic.c_str()) == 0) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        slam1_recive_ = true;
        payload_slam1_.CopyFrom(payload);
    }
    if (strcmp(topic_name, slam_2_webots_base_topic.c_str()) == 0) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        slam2_recive_ = true;
        payload_slam2_.CopyFrom(payload);
    }
    if (strcmp(topic_name, slam_3_webots_base_topic.c_str()) == 0) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        slam3_recive_ = true;
        payload_slam3_.CopyFrom(payload);
    }

    // 三帧拼一帧
    if (slam1_recive_ && slam2_recive_ && slam3_recive_) {
        // 读写锁
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        // 拷贝
        sim_data_flow::WBPointCloud payload_result;
        payload_result.CopyFrom(payload_slam1_);
        payload_result.mutable_point_cloud()->MergeFrom(payload_slam2_.point_cloud());
        payload_result.mutable_point_cloud()->MergeFrom(payload_slam3_.point_cloud());

        // 填入时间戳
        uint64_t time_stamp = 0;
        time_stamp = std::min(payload_slam1_.timestamp(), payload_slam2_.timestamp());
        time_stamp = std::min(time_stamp, payload_slam3_.timestamp());
        payload_result.set_timestamp(time_stamp);
        std::cout<<"time_stamp "<<time_stamp<<std::endl;

        // 转pointcloud2
        pb::PointCloud2 payload_send;
        pbTopb2(payload_result, payload_send, seq++);
        uint8_t buf[payload_send.ByteSize()];
        payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
        ecal_ptr_->send(multi_mid360_topic.c_str(), buf, payload_send.ByteSize());

        slam1_recive_ = false;
        slam2_recive_ = false;
        slam3_recive_ = false;

        payload_slam1_.Clear();
        payload_slam2_.Clear();
        payload_slam3_.Clear();
    }
}