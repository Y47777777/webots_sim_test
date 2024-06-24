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
    void onBpMsg(const char *topic_name,
                 const eCAL::SReceiveCallbackData *data);
    void onMid360Msg(const char *topic_name,
                     const eCAL::SReceiveCallbackData *data);
    void onMid360TwoMsg(const char *topic_name,
                        const eCAL::SReceiveCallbackData *data);

   private:
    uint64_t seq_bp_{0};
    uint64_t seq_mid360_{0};
};
}  // namespace VNSim

#endif