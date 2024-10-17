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

std::string lidar_2_webots_topic = "webots/Lidar/.111/PointCloud";
std::string lidar_4_webots_topic = "webots/Lidar/.113/PointCloud";
std::string lidar_3_webots_topic = "webots/Lidar/.112/PointCloud";
std::string lidar_0_webots_topic = "webots/Lidar/.109/PointCloud";
std::string be_webots_topic = "webots/Lidar/.200/PointCloud";
std::string bp_webots_topic = "webots/Lidar/.55/PointCloud";

std::string lidar_2_real_topic = "192.168.1.111";
std::string lidar_4_real_topic = "192.168.1.113";
std::string lidar_3_real_topic = "192.168.1.112";
std::string lidar_0_real_topic = "192.168.1.109";
std::string be_real_topic = "192.168.1.200";
std::string bp_real_topic = "192.168.1.55";

SVCShadow::SVCShadow() : BaseSvc() {}

SVCShadow::~SVCShadow() {}

int SVCShadow::initService() {
    // pub
    ecal_ptr_->addEcal(lidar_2_real_topic.c_str());
    ecal_ptr_->addEcal(lidar_4_real_topic.c_str());
    ecal_ptr_->addEcal(lidar_3_real_topic.c_str());
    ecal_ptr_->addEcal(lidar_0_real_topic.c_str());
    ecal_ptr_->addEcal(be_real_topic.c_str());
    ecal_ptr_->addEcal(bp_real_topic.c_str());

    // sub
    // TODO: 可以用匿名函数套一手
    ecal_ptr_->addEcal(lidar_2_webots_topic.c_str(),
                       std::bind(&SVCShadow::onLidar2Mssg, this, std::placeholders::_1, std::placeholders::_2));
    ecal_ptr_->addEcal(lidar_4_webots_topic.c_str(),
                       std::bind(&SVCShadow::onLidar4Mssg, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_3_webots_topic.c_str(),
                       std::bind(&SVCShadow::onLidar3Mssg, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_0_webots_topic.c_str(),
                       std::bind(&SVCShadow::onLidar0Mssg, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(be_webots_topic.c_str(),
                       std::bind(&SVCShadow::onBrighteyeMsg, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_2_webots_base_topic.c_str(),
                       std::bind(&SVCShadow::onMultiMid360Msg, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_4_webots_base_topic.c_str(),
                       std::bind(&SVCShadow::onMultiMid360Msg, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_3_webots_base_topic.c_str(),
                       std::bind(&SVCShadow::onMultiMid360Msg, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(bp_webots_topic.c_str(),
                       std::bind(&SVCShadow::onBPMsg, this, std::placeholders::_1, std::placeholders::_2));

    return 0;
}

// TODO: 是不是可以归一成一条函数(如何避免多线程？)
void SVCShadow::onLidar2Mssg(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;
    payload.ParseFromArray(data->buf, data->size);
    pbTopb2(payload, payload_send, seq++, 2);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(lidar_2_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onLidar4Mssg(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq++, 3);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(lidar_4_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onLidar3Mssg(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;

    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq++, 4);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(lidar_3_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onLidar0Mssg(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;

    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq++, 0);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(lidar_0_real_topic.c_str(), buf, payload_send.ByteSize());
}

void SVCShadow::onBrighteyeMsg(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
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

void SVCShadow::onBPMsg(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    static uint64_t seq = 0;

    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, seq++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send(bp_real_topic.c_str(), buf, payload_send.ByteSize());
}
