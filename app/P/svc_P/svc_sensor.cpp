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

std::string BP_webots_topic = "webots/Lidar/.55/PointCloud";
std::string MID360_webots_topic = "webots/Lidar/.54/PointCloud";
std::string MID360Two_webots_topic = "webots/Lidar/.56/PointCloud";
std::string MID360Per_webots_topic = "webots/Lidar/.57/PointCloud";
std::string HAP_webots_topic = "webots/Lidar/.200/PointCloud";

std::string BP_real_topic = "192.168.1.55";
std::string MID360_real_topic = "192.168.1.54";
std::string MID360Two_real_topic = "192.168.1.56";
std::string MID360Per_real_topic = "192.168.1.100";
std::string HAP_real_topic = "192.168.1.200";

std::string lidar_2_webots_topic = "webots/Lidar/.111/PointCloud";
std::string lidar_4_webots_topic = "webots/Lidar/.113/PointCloud";
std::string lidar_3_webots_topic = "webots/Lidar/.112/PointCloud";
std::string lidar_0_webots_topic = "webots/Lidar/.109/PointCloud";

std::string lidar_2_real_topic = "192.168.1.111";
std::string lidar_4_real_topic = "192.168.1.113";
std::string lidar_3_real_topic = "192.168.1.112";
std::string lidar_0_real_topic = "192.168.1.109";

SVCShadow::SVCShadow() : BaseSvc() {}

SVCShadow::~SVCShadow() {}

int SVCShadow::initService() {
    // pub
    ecal_ptr_->addEcal(BP_real_topic.c_str());
    ecal_ptr_->addEcal(lidar_2_real_topic.c_str());  // Mid360
    ecal_ptr_->addEcal(lidar_4_real_topic.c_str());  // Mid360Two
    ecal_ptr_->addEcal(lidar_0_real_topic.c_str());  // Mid360Two
    ecal_ptr_->addEcal(HAP_real_topic.c_str());      // Mid360Two

    // sub
    // TODO: 可以用匿名函数套一手
    // ecal_ptr_->addEcal(BP_webots_topic.c_str(),
    //                    std::bind(&SVCShadow::onBpMsg, this,
    //                              std::placeholders::_1,
    //                              std::placeholders::_2));
    ecal_ptr_->addEcal(lidar_2_webots_topic.c_str(),
                       std::bind(&SVCShadow::onMid360Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_4_webots_topic.c_str(),
                       std::bind(&SVCShadow::onMid360TwoMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_0_webots_topic.c_str(),
                       std::bind(&SVCShadow::onMid360PerMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(HAP_webots_topic.c_str(),
                       std::bind(&SVCShadow::onHapMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    return 0;
}

// TODO: 是不是可以归一成一条函数(如何避免多线程？)
void SVCShadow::onMid360Msg(const char *topic_name,
                            const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq_mid360_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(lidar_2_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onMid360TwoMsg(const char *topic_name,
                               const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq_mid360_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(lidar_4_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onMid360PerMsg(const char *topic_name,
                               const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq_mid360_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(lidar_0_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onBpMsg(const char *topic_name,
                        const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);
    pbTopb2(payload, payload_send, seq_bp_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(BP_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onHapMsg(const char *topic_name,
                         const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    // LOG_INFO("111111111111111111111");
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);
    pbTopb2(payload, payload_send, seq_bp_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(HAP_real_topic.c_str(), buf, payload_send.ByteSize());
}