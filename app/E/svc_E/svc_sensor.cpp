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
#include "dataTransform.h"
#include "svc_sensor.h"
#include "sim_data_flow/point_cloud2.pb.h"
#include "sim_data_flow/point_cloud.pb.h"

using namespace VNSim;

std::string slam_1_webots_topic = "webots/Lidar/.54/PointCloud";
std::string slam_2_webots_topic = "webots/Lidar/.56/PointCloud";
std::string slam_3_webots_topic = "webots/Lidar/.57/PointCloud";
std::string perception_webots_topic = "webots/Lidar/.100/PointCloud";

std::string slam_1_real_topic = "192.168.1.54";
std::string slam_2_real_topic = "192.168.1.56";
std::string slam_3_real_topic = "192.168.1.57";
std::string perception_real_topic = "192.168.1.100";

SVCShadow::SVCShadow() : BaseSvc() {}

SVCShadow::~SVCShadow() {}

int SVCShadow::initService() {
    // pub
    ecal_ptr_->addEcal(slam_1_real_topic.c_str());
    ecal_ptr_->addEcal(slam_2_real_topic.c_str());     // Mid360
    ecal_ptr_->addEcal(slam_3_real_topic.c_str());  // Mid360Two
    ecal_ptr_->addEcal(perception_real_topic.c_str());  // Mid360Two

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

    return 0;
}

// TODO: 是不是可以归一成一条函数(如何避免多线程？)
void SVCShadow::onSlam1Msg(const char *topic_name,
                        const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);
    pbTopb2(payload, payload_send, seq_bp_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(slam_1_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onSlam2Msg(const char *topic_name,
                            const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq_mid360_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(slam_2_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onSlam3Msg(const char *topic_name,
                               const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq_mid360_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(slam_3_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onPerceptionMsg(const char *topic_name,
                               const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq_mid360_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(perception_real_topic.c_str(), buf, payload_send.ByteSize());
}

