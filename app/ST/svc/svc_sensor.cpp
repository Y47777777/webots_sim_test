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
std::string lidar_3_webots_topic = "webots/Lidar/.112/PointCloud";
std::string lidar_0_webots_topic = "webots/Lidar/.109/PointCloud";
std::string be_webots_topic = "webots/Lidar/.200/PointCloud";

std::string lidar_2_webots_base_topic = "webots/LidarToBase/.111/PointCloud";
std::string lidar_3_webots_base_topic = "webots/LidarToBase/.112/PointCloud";

std::string lidar_2_real_topic = "192.168.1.111";
std::string lidar_3_real_topic = "192.168.1.112";
std::string lidar_0_real_topic = "192.168.1.109";
std::string be_real_topic = "192.168.1.200";

std::string multi_mid360_topic = "multi_mid360";

SVCShadow::SVCShadow() : BaseSvc() {}

SVCShadow::~SVCShadow() {}

int SVCShadow::initService() {
    // pub
    ecal_ptr_->addEcal(lidar_2_real_topic.c_str());
    ecal_ptr_->addEcal(lidar_3_real_topic.c_str());
    ecal_ptr_->addEcal(lidar_0_real_topic.c_str());
    ecal_ptr_->addEcal(be_real_topic.c_str());
    ecal_ptr_->addEcal(multi_mid360_topic.c_str());

    // sub
    // TODO: 可以用匿名函数套一手
    ecal_ptr_->addEcal(lidar_2_webots_topic.c_str(),
                       std::bind(&SVCShadow::onLidar2Mssg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_3_webots_topic.c_str(),
                       std::bind(&SVCShadow::onLidar3Mssg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_0_webots_topic.c_str(),
                       std::bind(&SVCShadow::onLidar0Mssg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(be_webots_topic.c_str(),
                       std::bind(&SVCShadow::onBrighteyeMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // MultiMid360
    ecal_ptr_->addEcal(lidar_2_webots_base_topic.c_str(),
                       std::bind(&SVCShadow::onMultiMid360Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal(lidar_3_webots_base_topic.c_str(),
                       std::bind(&SVCShadow::onMultiMid360Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    return 0;
}

// TODO: 是不是可以归一成一条函数(如何避免多线程？)
void SVCShadow::onLidar2Mssg(const char *topic_name,
                             const eCAL::SReceiveCallbackData *data) {
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

void SVCShadow::onLidar3Mssg(const char *topic_name,
                             const eCAL::SReceiveCallbackData *data) {
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

void SVCShadow::onLidar0Mssg(const char *topic_name,
                             const eCAL::SReceiveCallbackData *data) {
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
    pb::PointCloud2 payload_pc2;
    static uint64_t seq = 0;
    payload.ParseFromArray(data->buf, data->size);

    // 拷贝至缓存
    static std::shared_mutex rw_mutex_;  // 读写锁
    static bool lidar2_recive_ = false;
    static bool lidar3_recive_ = false;
    static pb::PointCloud2 payload_lidar2_;
    static pb::PointCloud2 payload_lidar3_;

    if (strcmp(topic_name, lidar_2_webots_base_topic.c_str()) == 0) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        lidar2_recive_ = true;
        pbTopb2(payload, payload_pc2, seq++, 2);
        payload_lidar2_.CopyFrom(payload_pc2);
    }
    if (strcmp(topic_name, lidar_3_webots_base_topic.c_str()) == 0) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        lidar3_recive_ = true;
        pbTopb2(payload, payload_pc2, seq++, 4);
        payload_lidar3_.CopyFrom(payload_pc2);
    }

    // 三帧拼一帧
    if (lidar2_recive_ && lidar3_recive_) {
        // 拷贝
        pb::PointCloud2 payload_result;
        {
            std::unique_lock<std::shared_mutex> lock(rw_mutex_);
            // lidar2 (主雷达)
            payload_result.CopyFrom(payload_lidar2_);

            // lidar3 左
            payload_result.mutable_data()->append(payload_lidar3_.data());
            payload_result.set_width(payload_result.width() +
                                     payload_lidar3_.width());
            payload_result.set_row_step(payload_result.row_step() +
                                        payload_lidar3_.row_step());

            // 填入时间戳
            uint64_t time_stamp = 0;
            time_stamp = std::min(payload_lidar2_.header().timestamp(),
                                  payload_lidar3_.header().timestamp());
        }

        uint8_t buf[payload_result.ByteSize()];
        payload_result.SerializePartialToArray(buf, payload_result.ByteSize());
        ecal_ptr_->send(multi_mid360_topic.c_str(), buf,
                        payload_result.ByteSize());

        lidar2_recive_ = false;
        lidar3_recive_ = false;

        payload_lidar2_.Clear();
        payload_lidar3_.Clear();
    }
}