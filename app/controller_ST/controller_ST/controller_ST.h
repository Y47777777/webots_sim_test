#ifndef __WEBOTST_HPP__
#define __WEBOTST_HPP__
#include <thread>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Emitter.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/VacuumGripper.hpp>

#include "base_ctrl.h"
#include "point_cloud2.pb.h"

using namespace webots;

class NormalSTController : public BaseController {
   private:
    /* Webot Param */
    /* Wheel */
    Supervisor *supervisor_;

    Motor *SteerWheelMotor_{nullptr};
    PositionSensor *SteerWheelPositionSensor_{nullptr};
    Node *steerNode_{nullptr};

    /* Fork */
    Motor *forkMotor_{nullptr};
    PositionSensor *forkPositionSensor_{nullptr};

    /* Lidar */
    Lidar *BP_{nullptr};
    Node *mid360Node_{nullptr};
    Lidar *mid360_{nullptr};
    /*IMU*/
    InertialUnit *inertialUnit_{nullptr};
    Gyro *gyro_{nullptr};
    Accelerometer *accelerometer_{nullptr};

   private:
    bool webotsExited_{false};

   protected:
    /*Simulation Param*/
    int wheel_state_;
    int fork_state_;
    int total_state_;
    uint32_t dataidx_upload_{0};
    uint32_t dataIdx_{0};
    uint64_t lidar_count_{0};
    double steerSpeed_{0};
    double steerYaw_{0};
    double vehicleYaw_{0};
    double increamentYaw_{0};
    double forkSpeed_{0};
    double forkPositionSensor_val_{0};
    double realSpeed_{0};
    double mid360InitPose_[3] = {0, 0, 0};
    double inertial_angle_value_[3] = {0, 0, 0};
    double inertial_angular_velocity_[3] = {0, 0, 0};
    double acc_meter_value_[3] = {0, 0, 0};
    double push_up_height_{1.36};
    double pos_{0};
    double RPM_{0};
    struct wheelCoder {
        double wheel_l_code{0.};
        double wheel_r_code{0.};
    };
    wheelCoder mWheelCoder_;

    const int MillStep_5 = 5;
    const int MillStep_10 = 10;
    const int MillStep_50 = 50;
    const int MillStep_100 = 100;
    std::map<std::string, double> keyboard_input_;
    std::map<std::string, double> keyboard_output_;

   public:
    NormalSTController();
    ~NormalSTController();
    void PointCloud2Init(pb::PointCloud2 &pb, int size);

   private:
    void writeSteerWheel();
    void setSteerWheelYaw(double yaw);
    void writeFork();
    void enableMotor();
    void enableLidar3D();
    void enableIMU();
    void updateVehicleYaw();
    void updateRPM();
    void updateInertialUnit();
    void updateForkState();
    void onRemoteSpeedChange(double steerSpeed, double steerYaw,
                             double forkSpeed);
    void onManualSpeedChange(double steerSpeed, double steerYaw,
                             double forkSpeed, double pushUpHeight = -1);
    bool _readDeltaXYZ();

   private:
    std::thread report_bp_thread_;
    std::thread report_mid360_thread_;
    std::thread report_sencer_thread_;

   public:
    virtual int init();
    virtual void runSimulation();

   public:
    /*Manual Control*/
    virtual void onManualMsg(const char *msg);

    virtual void onManualReport(std::string &msg);
    /*Auto Control*/
    virtual void onRemoteMsg(
        uint8_t *msg,
        int len);  // TODO: This value will be changed,
                   // since the encode and decode processes are in svc_model_ST
};

#endif
