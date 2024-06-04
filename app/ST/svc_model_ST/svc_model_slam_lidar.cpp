#include "sim_data_flow/point_cloud2.pb.h"
#include "svc_model_slam_lidar.h"

using namespace VNSim;

SVCModelLidarSlam::SVCModelLidarSlam() : BaseSVCModel() {}

SVCModelLidarSlam::~SVCModelLidarSlam() {}

int SVCModelLidarSlam::initService() {
    ecal_wrapper_.addEcal(
        "webot/pointCloud",
        std::bind(&SVCModelLidarSlam::onLidarMsg, this, std::placeholders::_1,
                  std::placeholders::_2));
    ecal_wrapper_.addEcal("192.168.1.55");
    return 0;
}

void SVCModelLidarSlam::onLidarMsg(const char *topic_name,
                                   const eCAL::SReceiveCallbackData *data) {
    // directly send the msg to slam..., try...
    pb::PointCloud2 payload;
    payload.ParseFromArray(data->buf, data->size);
    payload.mutable_header()->set_timestamp(timestamp_generator_.timestamp());
    if (payload.ByteSize() > DEFAULT_LIDAR_MSG) {
        return;
    }
    payload.SerializePartialToArray(buf_, payload.ByteSize());
    ecal_wrapper_.send("192.168.1.55", buf_, payload.ByteSize());
}