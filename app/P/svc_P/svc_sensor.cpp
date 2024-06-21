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

using namespace VNSim;
#define MAXIMUM_BP_UPLOAD 28800
#define MAXIMUM_MID360_UPLOAD 20722

SVCModelLidar::SVCModelLidar() : BaseSVCModel() {}

SVCModelLidar::~SVCModelLidar() {}

int SVCModelLidar::initService() {
    ecal_ptr_->addEcal("webot/pointCloud",
                       std::bind(&SVCModelLidar::onBpMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));
    ecal_ptr_->addEcal("webot/perception",
                       std::bind(&SVCModelLidar::onMid360Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/perceptionTwo",
                       std::bind(&SVCModelLidar::onMid360TwoMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("192.168.1.55");
    ecal_ptr_->addEcal("192.168.1.54"); //Mid360
    ecal_ptr_->addEcal("192.168.1.56"); //Mid360Two
    return 0;
}

void SVCModelLidar::onMid360Msg(const char *topic_name,
                                const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, MAXIMUM_MID360_UPLOAD, seq_mid360_++);
uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send("192.168.1.54", buf, payload_send.ByteSize());
}

void SVCModelLidar::onMid360TwoMsg(const char *topic_name,
                                const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, MAXIMUM_MID360_UPLOAD, seq_mid360_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send("192.168.1.56", buf, payload_send.ByteSize());
}

void SVCModelLidar::onBpMsg(const char *topic_name,
                            const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);
    pbTopb2(payload, payload_send, MAXIMUM_BP_UPLOAD, seq_bp_++);
    uint8_t buf[payload_send.ByteSize()];
    payload_send.SerializePartialToArray(buf, payload_send.ByteSize());
    ecal_ptr_->send("192.168.1.55", buf, payload_send.ByteSize());
}