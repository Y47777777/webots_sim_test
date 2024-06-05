#ifndef __SVC_MODEL_SLAM_LIDAR_H__
#define __SVC_MODEL_SLAM_LIDAR_H__
#define DEFAULT_LIDAR_MSG 900000

#include "time/time.h"
#include "sim_data_flow/point_cloud2.pb.h"
#include "sim_data_flow/point_cloud.pb.h"
#include "svc/base_svc_ctrl.h"

namespace VNSim {
struct PointFieldBw {
    std::string name;
    uint32_t bt_width;
    uint32_t datatype;
    uint32_t count;
};
// This is for BP
class SVCModelLidarSlam : public BaseSVCModel {
   public:
    SVCModelLidarSlam();
    ~SVCModelLidarSlam();

   public:
    int initService();

   public:
    void onLidarMsg(const char *topic_name,
                    const eCAL::SReceiveCallbackData *data);

   private:
    uint8_t buf_[DEFAULT_LIDAR_MSG];
    int size_;
    uint32_t maximum_upload_point_;
    FixedTimeTimestampGenerator timestamp_generator_{
        10, 1640966400000000 + 8 * 3600 + 1000000};
    uint64_t seq_{0};
    std::vector<PointFieldBw> bpPointField_{
        {"x", 4, 7, 1},         {"y", 4, 7, 1},     {"z", 4, 7, 1},
        {"intensity", 4, 7, 1}, {"label", 4, 6, 1}, {"timestamp", 8, 8, 1}};

   private:
    void transformData(const sim_data_flow::WBPointCloud &payload,
                       pb::PointCloud2 &pb);
};
}  // namespace VNSim

#endif