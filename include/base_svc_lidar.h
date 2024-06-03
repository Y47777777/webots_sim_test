#ifndef __BASE_SVC_LIDAR_H__
#define __BASE_SVC_LIDAR_H__

#include "base_svc_serial.h"
#include "base_svc_ctrl.h"

namespace VNSim {
class BaseLidarSVCModel : public BaseSVCModel {
   public:
    BaseLidarSVCModel() : BaseSVCModel(), serial_ptr_(nullptr) {}
    ~BaseLidarSVCModel() {}

   protected:
    std::shared_ptr<BaseSVCModel> serial_ptr_;

   public:
    int initService() { return this->onInitService(); }
    void addSerialPtr(std::shared_ptr<BaseSVCModel> ptr) { serial_ptr_ = ptr; }

   public:
    virtual int onInitService() = 0;
};
}  // namespace VNSim

#endif