/**
 * @file svc_ctrl.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-06-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __SVC_MODEL_SERIAL_H__
#define __SVC_MODEL_SERIAL_H__

#include "sim_data_flow/E40_msg.pb.h"
#include "svc/base_svc_serial.h"

namespace VNSim {

class SVCMaster : public BaseSerialSvc {
    // TODO: 接口说明
   public:
    SVCMaster();
    ~SVCMaster();

   private:
    int onInitService();
    void subDownStreamCallBack(uint8_t *msg, int len);
    void subEMsgCallBack(const char *topic_name,
                         const eCAL::SReceiveCallbackData *data);
    void subPoseCallBakc(const char *topic_name,
                         const eCAL::SReceiveCallbackData *data);

    void pubUpStream();
    void pubEMsgsToWebots();
   private:
    void onWatchDogHungry() override;

   private:
    sim_data_flow::E40MsgUp msg_from_webots_;
    sim_data_flow::E40MsgDown msg_to_webots_;
    struct Package msgs_from_agv_;
};
}  // namespace VNSim

#endif