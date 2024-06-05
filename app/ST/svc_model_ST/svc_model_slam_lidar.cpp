#include <math.h>
#include "svc_model_slam_lidar.h"

using namespace VNSim;
#define MAXIMUM_BP_UPLOAD 28800
#define MAXIMUM_MID360_UPLOAD 20722
#define PBPOINT_BANDWIDTH 4 * 5 + 8

SVCModelLidar::SVCModelLidar() : BaseSVCModel() {}

SVCModelLidar::~SVCModelLidar() {}

int SVCModelLidar::initService() {
    ecal_wrapper_.addEcal(
        "webot/pointCloud",
        std::bind(&SVCModelLidar::onBpMsg, this, std::placeholders::_1,
                  std::placeholders::_2));
    ecal_wrapper_.addEcal(
        "webot/perception",
        std::bind(&SVCModelLidar::onMid360Msg, this, std::placeholders::_1,
                  std::placeholders::_2));
    ecal_wrapper_.addEcal("192.168.1.55");
    ecal_wrapper_.addEcal("mid360pub");
    return 0;
}

void SVCModelLidar::transformData(const sim_data_flow::WBPointCloud &payload,
                                  pb::PointCloud2 &payload_send,
                                  uint32_t total_lidar_point) {
    // header
    payload_send.mutable_header()->set_frame_id("");
    payload_send.mutable_header()->set_seq(seq_++);
    payload_send.mutable_header()->set_timestamp(payload.timestamp());
    // body
    payload_send.set_height(1);
    payload_send.set_width(total_lidar_point);
    payload_send.set_point_step(PBPOINT_BANDWIDTH);
    payload_send.set_row_step(PBPOINT_BANDWIDTH * total_lidar_point);
    payload_send.set_is_bigendian(false);
    payload_send.set_is_dense(false);
    // field
    // payload_send.clear_fields();
    uint32_t offset = 0;
    for (size_t i = 0; i < PointField_.size(); i++) {
        auto field = payload_send.add_fields();
        field->set_name(PointField_[i].name);
        field->set_datatype(PointField_[i].datatype);
        if (i > 0)
            offset += PointField_[i - 1].bt_width;
        field->set_offset(offset);
        field->set_count(PointField_[i].count);
    }
    // data
    payload_send.mutable_data()->resize(PBPOINT_BANDWIDTH * total_lidar_point);
    uint64_t point_size = payload.size_of_point_cloud();
    char *pb_data_ptr = &((*payload_send.mutable_data())[0]);
    int intensity = 130;
    int label = 8;
    for (int i = 0; i < point_size; i++) {
        double x = payload.point_cloud().at(i).x();
        double y = payload.point_cloud().at(i).y();
        double z = payload.point_cloud().at(i).z();
        double time = payload.point_cloud().at(i).time();
        if (std::abs(x) != INFINITY && std::abs(y) != INFINITY &&
            std::abs(z) != INFINITY) {
            if (i < total_lidar_point - 1) {
                // 4 * 5 + 8
                memcpy(pb_data_ptr + i * PBPOINT_BANDWIDTH, &(x), 4);
                memcpy(pb_data_ptr + 4 + i * PBPOINT_BANDWIDTH, &(y), 4);
                memcpy(pb_data_ptr + 8 + i * PBPOINT_BANDWIDTH, &(z), 4);
                memcpy(pb_data_ptr + 12 + i * PBPOINT_BANDWIDTH, &(intensity),
                       4);
                memcpy(pb_data_ptr + 16 + i * PBPOINT_BANDWIDTH, &(label), 4);
                memcpy(pb_data_ptr + 20 + i * PBPOINT_BANDWIDTH, &(time), 8);
            } else {
                // No more any points
                break;
            }
        }
    }
}

void SVCModelLidar::onMid360Msg(const char *topic_name,
                                const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);
    this->transformData(payload, payload_send, MAXIMUM_MID360_UPLOAD);
    payload_send.SerializePartialToArray(buf2_, payload_send.ByteSize());
    ecal_wrapper_.send("mid360pub", buf2_, payload_send.ByteSize());
}
void SVCModelLidar::onBpMsg(const char *topic_name,
                            const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);
    this->transformData(payload, payload_send, MAXIMUM_BP_UPLOAD);
    payload_send.SerializePartialToArray(buf_, payload_send.ByteSize());
    ecal_wrapper_.send("192.168.1.55", buf_, payload_send.ByteSize());
}