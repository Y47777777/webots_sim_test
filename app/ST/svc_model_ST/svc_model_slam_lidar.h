#ifndef __SVC_MODEL_SLAM_LIDAR_H__
#define __SVC_MODEL_SLAM_LIDAR_H__
#define DEFAULT_LIDAR_MSG 900000

#include "time/time.h"
#include "svc/base_svc_ctrl.h"

namespace VNSim {
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
    FixedTimeTimestampGenerator timestamp_generator_{
        10, 1640966400000000 + 8 * 3600 + 1000000};
};
}  // namespace VNSim

#endif