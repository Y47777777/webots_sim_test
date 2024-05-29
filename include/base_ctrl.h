#ifndef __BASE_CTRL_H__
#define __BASE_CTRL_H__

#include <vector>
#include <map>
#include <QThread>
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

namespace VNSim {

class BaseController : public QThread {
   public:
    explicit BaseController(QObject *parent = nullptr) : QThread{parent} {}

    ~BaseController() {
        // 由base管理线程
        for (auto it = m_thread_.begin(); it != m_thread_.end(); ++it) {
            std::thread &thread = it->second;
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

    void run() {
        this->init();
        this->whileSpin();
    }
    virtual void setRobotState(const std::map<std::string, double> &msg) {}
    virtual void getRobotState(std::map<std::string, double> &msg) {}
    void shiftControlMode(bool mode) { isManual_ = mode; }

   protected:
    // state interface
    virtual void getSteerWheelState() = 0;
    virtual void setSteerWheelState() = 0;
    virtual void setForkState() = 0;
    virtual void getForkState() = 0;
    virtual void getIMUState() = 0;

    // spin task
    virtual void init() = 0;
    virtual void whileSpin() = 0;

   protected:
    bool isManual_ = false;
    // webots
    webots::Supervisor *supervisor_ = nullptr;

    webots::Motor *steer_motor_ptr_ = nullptr;
    webots::PositionSensor *steer_pos_sensor_ptr_ = nullptr;
    webots::Node *steer_node_ptr_ = nullptr;
    webots::Node *steerwheel_node_ptr_ = nullptr;

    /* Fork */
    webots::Motor *fork_motor_ptr_ = nullptr;
    webots::PositionSensor *fork_pos_sensor_ptr_ = nullptr;

    /*IMU*/
    webots::InertialUnit *inertial_unit_ptr_ = nullptr;
    webots::Gyro *gyro_ptr_ = nullptr;
    webots::Accelerometer *accelerometer_ptr_ = nullptr;

    bool webotsExited_ = false;

    std::map<std::string, std::thread> m_thread_;
};

}  // namespace VNSim

#endif
