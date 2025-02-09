/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 13:23:56
 * @FilePath: /webots_ctrl/app/ST/svc_model_ST/svc_model_slam_lidar.cpp
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#include "dataTransform.h"
#include "svc_model_slam_lidar.h"

using namespace VNSim;
#define MAXIMUM_BP_UPLOAD 28800
#define MAXIMUM_MID360_UPLOAD 20722

SVCShadow::SVCShadow() : BaseSvc() {}

SVCShadow::~SVCShadow() {}

int SVCShadow::initService() {
    ecal_ptr_->addEcal("webot/pointCloud",
                       std::bind(&SVCShadow::onBpMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));
    ecal_ptr_->addEcal("webot/perception",
                       std::bind(&SVCShadow::onMid360Msg, this,
                                 std::placeholders::_1, std::placeholders::_2));
    ecal_ptr_->addEcal("webot/perception1",
                       std::bind(&SVCShadow::onMid360TwoMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("192.168.1.55");
    ecal_ptr_->addEcal("mid360pub");
    ecal_ptr_->addEcal("mid360pub1");
    return 0;
}

void SVCShadow::onMid360Msg(const char *topic_name,
                                const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, MAXIMUM_MID360_UPLOAD, seq_mid360_++);

    payload_send.SerializePartialToArray(buf2_, payload_send.ByteSize());
    ecal_ptr_->send("mid360pub", buf2_, payload_send.ByteSize());
}

void SVCShadow::onMid360TwoMsg(const char *topic_name,
                                   const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    // 转pointcloud2
    pbTopb2(payload, payload_send, MAXIMUM_MID360_UPLOAD, seq_mid360_++);

    uint8_t buf2[DEFAULT_LIDAR_MSG];
    payload_send.SerializePartialToArray(buf2, payload_send.ByteSize());
    ecal_ptr_->send("mid360pub1", buf2, payload_send.ByteSize());
}

void SVCShadow::onBpMsg(const char *topic_name,
                            const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);

    pbTopb2(payload, payload_send, MAXIMUM_BP_UPLOAD, seq_bp_++);

    payload_send.SerializePartialToArray(buf_, payload_send.ByteSize());
    ecal_ptr_->send("192.168.1.55", buf_, payload_send.ByteSize());
}