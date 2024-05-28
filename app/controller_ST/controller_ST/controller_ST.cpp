#include "controller_ST.h"

using namespace VNSim;
using namespace webots;

NormalSTController::NormalSTController() {
    supervisor_ = Supervisor::getSupervisorInstance();

    enableMotor();
    enableLidar3D();
    enableIMU();
}
NormalSTController::~NormalSTController() {}

void NormalSTController::enableMotor() {
    // get ptr
    steer_motor_ptr_ = supervisor_->getMotor("FL");
    steer_pos_sensor_ptr_ = steer_motor_ptr_->getPositionSensor();
    steer_node_ptr_ = supervisor_->getFromDef("SteerSolid");

    fork_motor_ptr_ = supervisor_->getMotor("fork height motor");
    fork_pos_sensor_ptr_ = fork_motor_ptr_->getPositionSensor();

    // init
    steer_motor_ptr_->setPosition(INFINITY);
    steer_motor_ptr_->setVelocity(0);
    steer_pos_sensor_ptr_->enable(MILLSTEP_5);

    fork_motor_ptr_->setPosition(INFINITY);
    fork_motor_ptr_->setVelocity(0);
    fork_pos_sensor_ptr_->enable(MILLSTEP_5);
}

void NormalSTController::enableLidar3D() {
    // get ptr
    mid360_node_ptr_ = supervisor_->getFromDef("MID360");
    mid360_ptr_ = supervisor_->getLidar("mid360");
    BP_ptr_ = supervisor_->getLidar("BP");

    // init
    BP_ptr_->enable(MILLSTEP_5);
    BP_ptr_->enablePointCloud();

    mid360_ptr_->enable(MILLSTEP_5);
    mid360_ptr_->enablePointCloud();

    Field *mid360_tf = mid360_node_ptr_->getField("translation");
    const double *mid360_pose = mid360_tf->getSFVec3f();
    // TODO:
    // mid360InitPose_[0] = mid360_pose[0];
    // mid360InitPose_[1] = mid360_pose[1];
    // mid360InitPose_[2] = mid360_pose[2];
}

void NormalSTController::enableIMU() {
    // get ptr
    inertial_unit_ptr_ = supervisor_->getInertialUnit("inertial unit");
    gyro_prt_ = supervisor_->getGyro("gyro");
    accelerometer_ptr_ = supervisor_->getAccelerometer("accelerometer");

    // init
    inertial_unit_ptr_->enable(MILLSTEP_5);
    gyro_prt_->enable(MILLSTEP_5);
    accelerometer_ptr_->enable(MILLSTEP_5);
}

void NormalSTController::getSteerWheelState() {}

void NormalSTController::setSteerWheelState() {}

void NormalSTController::setForkState() {}

void NormalSTController::getForkState() {}

void NormalSTController::getIMUState() {}

void NormalSTController::setRobotState(
    const std::map<std::string, double> &msg) {}

void NormalSTController::getRobotState(std::map<std::string, double> &msg) {}

void NormalSTController::whileSpin() {}

void NormalSTController::lidarBPSpin() {}

void NormalSTController::lidarMid360Spin() {}

void NormalSTController::imuSpin() {}
