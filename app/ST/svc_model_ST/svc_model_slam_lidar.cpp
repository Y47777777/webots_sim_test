#include <math.h>
#include "svc_model_slam_lidar.h"

using namespace VNSim;
#define MAXIMUM_BP_UPLOAD 28800
#define PBPOINT_BANDWIDTH 4 * 5 + 8

SVCModelLidarSlam::SVCModelLidarSlam()
    : BaseSVCModel(), maximum_upload_point_(MAXIMUM_BP_UPLOAD) {}

SVCModelLidarSlam::~SVCModelLidarSlam() {}

int SVCModelLidarSlam::initService() {
    ecal_wrapper_.addEcal(
        "webot/pointCloud",
        std::bind(&SVCModelLidarSlam::onLidarMsg, this, std::placeholders::_1,
                  std::placeholders::_2));
    ecal_wrapper_.addEcal("192.168.1.55");
    return 0;
}

void SVCModelLidarSlam::transformData(
    const sim_data_flow::WBPointCloud &payload, pb::PointCloud2 &payload_send) {
    // header
    payload_send.mutable_header()->set_frame_id("");
    payload_send.mutable_header()->set_seq(seq_++);
    payload_send.mutable_header()->set_timestamp(payload.timestamp());
    // body
    payload_send.set_height(1);
    payload_send.set_width(maximum_upload_point_);
    payload_send.set_point_step(PBPOINT_BANDWIDTH);
    payload_send.set_row_step(PBPOINT_BANDWIDTH * maximum_upload_point_);
    payload_send.set_is_bigendian(false);
    payload_send.set_is_dense(false);
    // field
    // payload_send.clear_fields();
    uint32_t offset = 0;
    for (size_t i = 0; i < bpPointField_.size(); i++) {
        auto field = payload_send.add_fields();
        field->set_name(bpPointField_[i].name);
        field->set_datatype(bpPointField_[i].datatype);
        if (i > 0)
            offset += bpPointField_[i - 1].bt_width;
        field->set_offset(offset);
        field->set_count(bpPointField_[i].count);
    }
    // data
    payload_send.mutable_data()->resize(PBPOINT_BANDWIDTH *
                                        maximum_upload_point_);
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
            if (i < maximum_upload_point_ - 1) {
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

void SVCModelLidarSlam::onLidarMsg(const char *topic_name,
                                   const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    sim_data_flow::WBPointCloud payload;
    pb::PointCloud2 payload_send;
    payload.ParseFromArray(data->buf, data->size);
    this->transformData(payload, payload_send);
    payload_send.SerializePartialToArray(buf_, payload_send.ByteSize());
    ecal_wrapper_.send("192.168.1.55", buf_, payload_send.ByteSize());
}