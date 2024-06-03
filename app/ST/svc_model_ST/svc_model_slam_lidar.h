#ifndef __SVC_MODEL_SLAM_LIDAR_H__
#define __SVC_MODEL_SLAM_LIDAR_H__
#define DEFAULT_LIDAR_MSG 900000

#include "base_svc_lidar.h"

namespace VNSim {
class SVCModelLidarSlam : public BaseLidarSVCModel {
   public:
    SVCModelLidarSlam();
    ~SVCModelLidarSlam();

   public:
    int onInitService();

   public:
    void onLidarMsg(const char *topic_name,
                    const eCAL::SReceiveCallbackData *data);

   private:
    uint8_t buf_[DEFAULT_LIDAR_MSG];
    int size_;
};
}  // namespace VNSim

#endif