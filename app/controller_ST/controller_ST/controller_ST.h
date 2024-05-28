#ifndef __WEBOTST_HPP__
#define __WEBOTST_HPP__
#include <thread>

#include "base_ctrl.h"

namespace VNSim {

#define MILLSTEP_5 5

struct CommonVehicleParams {
    double steer_speed;
    double steer_yaw;
    double steer_position;
    double fork_speed;
    double fork_height;
    double vehicle_yaw;
    double real_speed;
};

struct IMU {
    double angle[3];
    double velocity[3];
    double acceleration[3];
};

struct ST_Internal_Param {
    int fork_state;
};

struct ST_Internal_Vehicle_Param {
    struct CommonVehicleParams common_vehicle_params;
    struct IMU imu;
    double rpm;
};

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
    void init();
    void whileSpin();
    // void lidarBPSpin();
    // void lidarMid360Spin();
    // publish_fn 这个应该放入至传感器类中(自定格式proto)
    // void lidarBPPublish();

    // subscribe
    // void subCallBack();

    //    private:
    // webots sensor
    // TODO: 抽象成类(.h)
    // webots::Lidar *BP_ptr_ = nullptr;
    // webots::Node *mid360_node_ptr_ = nullptr;
    // webots::Lidar *mid360_ptr_ = nullptr;
    //    private:
    //     std::unique_ptr<VNSimLidar::BPLidar> bp_ptr_;

   private:
    void setWebotSteerWheel(double yaw);
    void getVehicleYaw();
    // publisher
    struct ST_Internal_Vehicle_Param st_internal_vehicle_params_;
    // struct ST_Internal_Param st_internal_params_;
};

}  // namespace VNSim

#endif
