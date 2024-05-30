#include <ecal/msg/protobuf/publisher.h>
#include "geometry/geometry.h"
// #include "foxglove-vn/Imu.pb.h"       // IMU
// #include "foxglove-vn/ForkPose.pb.h"  // vehicle height
// #include "foxglove-vn/Vector2.pb.h"   // use for origin steer wheel position
#include "controller_ST.h"

#include "sim_data_flow/ST_msg.pb.h"

using namespace VNSim;
using namespace webots;

#define SIMULATION_STEP 2
// #define FORK_LIFTUP_HEIGHT 0.085  // TODO: it should be config
// #define FORK_LIFTDOWN_HEIGHT 0    // TODO: it should be config
// #define HEIGHT_DEVIATION 0.0001   // meter
// #define SIM_FAC 1.06695119

enum FORK_STATE { ON_FORK_BOTTOM = 0, ON_FORK_MIDDLE = 1, ON_FORK_TOP = 2 };

NormalSTController::NormalSTController() {
    supervisor_ = Supervisor::getSupervisorInstance();

    // sensor init
    BP_ptr_ = std::make_shared<WLidar>("BP");
    mid360_ptr_ = std::make_shared<WLidar>("mid360", "MID360");
    imu_ptr_ = std::make_shared<WImu>("inertial unit", "gyro", "accelerometer");

    // motor init
    fork_ptr_ = std::make_shared<WFork>("fork height motor");
    stree_ptr_ =
        std::make_shared<WWheel>("FL", "SteerWheel", "SteerSolid", "S");

    v_while_spin_.push_back(bind(&WBase::spin, stree_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, fork_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, imu_ptr_));
}
NormalSTController::~NormalSTController() {}

void NormalSTController::manualSetState(
    const std::map<std::string, double> &msg) {
    static double steer_speed = 0;
    static double steer_yaw = 0;
    static double fork_speed = 0;
    if (isManual_) {
        steer_speed = msg.at("steer_speed");
        steer_yaw = msg.at("steer_yaw");
        fork_speed = msg.at("fork_speed");

        stree_ptr_->setSpeed(steer_speed, steer_yaw);
        fork_ptr_->setVelocity(fork_speed);
    }
}

void NormalSTController::manualGetState(std::map<std::string, double> &msg) {
    msg["steer_speed"] = stree_ptr_->getSenosorValue();
    msg["steer_yaw"] = stree_ptr_->getMotorYaw();

    msg["fork_speed"] = fork_ptr_->getSenosorValue();
    msg["fork_height"] = fork_ptr_->getSenosorValue();
    msg["real_speed"] = fork_ptr_->getSenosorValue();

    // TODO: fork_speed real_speed
}

void NormalSTController::whileSpin() {
    // eCAL::protobuf::CPublisher<foxglove::ForkPose> pubFork("webot/forkpose");
    // eCAL::protobuf::CPublisher<foxglove::Imu> pubIMU("webot/IMUData");
    // eCAL::protobuf::CPublisher<foxglove::Vector2> pubWheelPosition(
    //     "webot/SteerWheelPosition");
    // foxglove::ForkPose payload_forkPose;
    // foxglove::Imu payload_imu;
    // foxglove::Vector2 payload_wheelPosition;


    webotsExited_ = false;
    while (supervisor_->step(SIMULATION_STEP) != -1) {
        for (int i = 0; i < v_while_spin_.size(); ++i) { v_while_spin_[i](); }

        sim_data_flow::ST_msg a;
    }
    webotsExited_ = true;
}
