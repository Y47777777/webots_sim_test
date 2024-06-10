/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-07 16:27:36
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 17:28:59
 * @FilePath: /webots_ctrl/app/ST/svc_model_ST/process_service_ST.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once

#include "data_process_service/ctrl_process.h"

namespace VNSim {
class STProcessService : public CtrlProcessBase {
   public:
    STProcessService();
    ~STProcessService();

   private:
    void fromCtrlSystem(uint8_t *msg, int len);
    void toCtrlSystem();
    void fromSimSystem(uint8_t *msg, int len);
    void toSimSystem();
};
}  // namespace VNSim