/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 13:19:30
 * @FilePath: /webots_ctrl/include/dataTransform.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#ifndef __DATA_TRANSFORM_H__
#define __DATA_TRANSFORM_H__
#include <vector>
#include <math.h>
#include "sim_data_flow/point_cloud2.pb.h"
#include "sim_data_flow/point_cloud.pb.h"
#include "lidar_simulation/high_reflector.h"
#include "time/time.h"

// TODO: data width 可以用sizeof(获取？)
#define PBPOINT_BANDWIDTH ((4 * 5) + 8)

namespace VNSim {
struct PointFieldBw {
    std::string name;
    uint32_t bt_width;
    uint32_t datatype;
    uint32_t count;
};

typedef union Label {
    union {
        struct {
            union {
                struct {
                    uint8_t tag;            // 雨雾信息： level 0->best,level 4 -> worst
                    uint8_t laser_type;     // 雷达标识 （雷达序号）
                };
                uint16_t label_u16;
            };
            uint16_t real;
        };
        uint32_t label_u32;
    };
};

/**
 * @brief pointCloud -> pointCloud2
 *
 * @param payload
 * @param payload_send
 * @param seq
 */
void pbTopb2(const sim_data_flow::WBPointCloud &payload,
             pb::PointCloud2 &payload_send, uint64_t seq, uint8_t laser_type = 0) {
    uint64_t point_size = payload.point_cloud().size();
    std::vector<PointFieldBw> PointField{
        {"x", 4, 7, 1},         {"y", 4, 7, 1},     {"z", 4, 7, 1},
        {"intensity", 4, 7, 1}, {"label", 4, 6, 1}, {"timestamp", 4, 7, 1}};
    // header
    payload_send.mutable_header()->set_frame_id("");
    payload_send.mutable_header()->set_seq(seq);
    payload_send.mutable_header()->set_timestamp(
        Timer::getInstance()->getTimeFromBase(payload.timestamp()));
    // payload_send.mutable_header()->set_timestamp();

    // body
    payload_send.set_height(1);
    payload_send.set_width(point_size);
    payload_send.set_point_step(PBPOINT_BANDWIDTH);
    payload_send.set_row_step(PBPOINT_BANDWIDTH * point_size);
    payload_send.set_is_bigendian(false);
    payload_send.set_is_dense(false);
    // field
    payload_send.clear_fields();

    uint32_t offset = 0;
    for (size_t i = 0; i < PointField.size(); i++) {
        auto field = payload_send.add_fields();
        field->set_name(PointField[i].name);
        field->set_datatype(PointField[i].datatype);
        if (i > 0)
            offset += PointField[i - 1].bt_width;
        field->set_offset(offset);
        field->set_count(PointField[i].count);
    }
    // data
    payload_send.mutable_data()->resize(PBPOINT_BANDWIDTH * (point_size));
    char *pb_data_ptr = &((*payload_send.mutable_data())[0]);

    // tag 赋值
    Label label;
    label.label_u32 = 0;
    label.laser_type = laser_type;

    // int label = 8;
    double point_base_time = 0;

    for (int i = 0; i < point_size; i++) {
        float x = payload.point_cloud().at(i).x();
        float y = payload.point_cloud().at(i).y();
        float z = payload.point_cloud().at(i).z();
        float intensity = payload.point_cloud().at(i).intensity();
        double time = payload.point_cloud().at(i).time();

        // 以帧的时间向上递增
        if (i == 0) {
            point_base_time = time;
        }
        time = time - point_base_time;
        time = time * 1000 * 1000;  // 换算成微秒
        time = time + (double) payload_send.header().timestamp();

        if (std::abs(x) != INFINITY && std::abs(y) != INFINITY &&
            std::abs(z) != INFINITY) {
            memcpy(pb_data_ptr + i * PBPOINT_BANDWIDTH, &(x), 4);
            memcpy(pb_data_ptr + 4 + i * PBPOINT_BANDWIDTH, &(y), 4);
            memcpy(pb_data_ptr + 8 + i * PBPOINT_BANDWIDTH, &(z), 4);
            memcpy(pb_data_ptr + 12 + i * PBPOINT_BANDWIDTH, &(intensity), 4);
            memcpy(pb_data_ptr + 16 + i * PBPOINT_BANDWIDTH, &(label), 4);
            memcpy(pb_data_ptr + 20 + i * PBPOINT_BANDWIDTH, &(time), 8);
        }
    }
}
}  // namespace VNSim

#endif