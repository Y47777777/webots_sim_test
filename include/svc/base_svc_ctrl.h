/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-07 17:39:42
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 20:03:33
 * @FilePath: /webots_ctrl/include/svc/base_svc_ctrl.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#ifndef __BASE_SVC_CTRL_H__
#define __BASE_SVC_CTRL_H__
#include <stdint.h>
#include <memory>
#include "ecal_wrapper/ecal_wrapper.h"
namespace VNSim {
class BaseSVCModel {
   protected:
    std::shared_ptr<EcalWrapper> ecal_ptr_;
    bool SVCExit_;

   public:
    BaseSVCModel() : SVCExit_(false) {}
    virtual ~BaseSVCModel() {}

   public:
    int init(bool OnInit, const char *name) {
        std::cout << "ecal init" << std::endl;
        ecal_ptr_ = EcalWrapper::getInstance(name);
        std::cout << "ecal init 2" << std::endl;
        return this->initService();
    }
    void svcExit() { SVCExit_ = true; }

   protected:
    virtual int initService() = 0;
};
}  // namespace VNSim

#endif