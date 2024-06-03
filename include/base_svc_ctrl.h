#ifndef __BASE_SVC_CTRL_H__
#define __BASE_SVC_CTRL_H__
#include <stdint.h>
#include <memory>
#include "ecal_wrapper.h"
namespace VNSim {
class BaseSVCModel {
   protected:
    EcalWrapper ecal_wrapper_;
    bool SVCExit_;

   public:
    BaseSVCModel() : SVCExit_(false) {}
    virtual ~BaseSVCModel() {}

   public:
    int init(bool OnInit, const char *name) {
        ecal_wrapper_.init(OnInit, name);
        return this->initService();
    }
    void svcExit() { SVCExit_ = true; }

   protected:
    virtual int initService() = 0;
};
}  // namespace VNSim

#endif