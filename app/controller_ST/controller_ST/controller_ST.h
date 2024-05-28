#ifndef __WEBOTST_HPP__
#define __WEBOTST_HPP__
#include <thread>

#include "base_ctrl.h"

namespace VNSim {

#define MILLSTEP_5 5

class NormalSTController : public BaseController {
   public:
    NormalSTController();
    ~NormalSTController();

    // Manual
    virtual void setRobotState(const std::map<std::string, double> &msg);
    virtual void getRobotState(std::map<std::string, double> &msg);

   protected:
    // init
    // TODO: 
    void enableMotor();
    void enableLidar3D();
    void enableIMU();

    // communicate whit webots
    void getSteerWheelState();
    void setSteerWheelState();
    void setForkState();
    void getForkState();
    void getIMUState();

    // task
    void whileSpin();
    void lidarBPSpin();
    void lidarMid360Spin();
    void imuSpin();

    // publish_fn 这个应该放入至传感器类中(自定格式proto)
    void lidarBPPublish();

    // subscribe
    void subCallBack();

   private:
    // webots sensor
    // TODO: 抽象成类(.h)
    webots::Lidar *BP_ptr_ = nullptr;
    webots::Node *mid360_node_ptr_ = nullptr;
    webots::Lidar *mid360_ptr_ = nullptr;

    // publisher
};

}  // namespace VNSim

#endif
