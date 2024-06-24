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

#include "sim_data_flow/P_msg.pb.h"
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
    void subPMsgCallBack(const char *topic_name,
                         const eCAL::SReceiveCallbackData *data);

    void pubUpStream();
    void pubPMsgsToWebots();

   private:
    sim_data_flow::PMsgUp   msg_from_webots_;
    sim_data_flow::PMsgDown msg_to_webots_;
    struct Package          msgs_from_agv_;

};
}  // namespace VNSim

#endif