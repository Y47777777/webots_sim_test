#ifndef __SVC_MODEL_SLAM_LIDAR_H__
#define __SVC_MODEL_SLAM_LIDAR_H__

#include "time/time.h"
#include "svc/base_svc.h"

namespace VNSim {

class SVCShadow : public BaseSvc {
   public:
    SVCShadow();
    ~SVCShadow();

   public:
    int initService();

   public:
    void onLidar2Mssg(const char *topic_name, const eCAL::SReceiveCallbackData *data);
    void onLidar4Mssg(const char *topic_name, const eCAL::SReceiveCallbackData *data);
    void onLidar3Mssg(const char *topic_name, const eCAL::SReceiveCallbackData *data);
    void onLidar0Mssg(const char *topic_name, const eCAL::SReceiveCallbackData *data);
    void onBrighteyeMsg(const char *topic_name, const eCAL::SReceiveCallbackData *data);

   private:
};
}  // namespace VNSim

#endif