#ifndef __SVC_MODEL_SLAM_LIDAR_H__
#define __SVC_MODEL_SLAM_LIDAR_H__
#define DEFAULT_LIDAR_MSG 900000

#include "time/time.h"
#include "sim_data_flow/point_cloud2.pb.h"
#include "sim_data_flow/point_cloud.pb.h"
#include "svc/base_svc.h"

namespace VNSim {
// This is for BP and mid360
class SVCShadow : public BaseSvc {
   public:
    SVCShadow();
    ~SVCShadow();

   public:
    int initService();

   public:
    void onBpMsg(const char *topic_name,
                 const eCAL::SReceiveCallbackData *data);
    void onMid360Msg(const char *topic_name,
                     const eCAL::SReceiveCallbackData *data);
    void onMid360TwoMsg(const char *topic_name,
                     const eCAL::SReceiveCallbackData *data);

   private:
    uint8_t buf_[DEFAULT_LIDAR_MSG];
    uint8_t buf2_[DEFAULT_LIDAR_MSG];
    uint64_t seq_bp_{0};
    uint64_t seq_mid360_{0};
};
}  // namespace VNSim

#endif