#include <ecal/msg/protobuf/publisher.h>
#include "utils.hpp"
#include "foxglove-vn/Imu.pb.h"       // IMU
#include "foxglove-vn/ForkPose.pb.h"  // vehicle height
#include "foxglove-vn/Vector2.pb.h"   // use for origin steer wheel position
#include "controller_ST.h"

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
    // mid360_node_ptr_ = supervisor_->getFromDef("MID360");
    // mid360_ptr_ = supervisor_->getLidar("mid360");
    // BP_ptr_ = supervisor_->getLidar("BP");

    // init
    // BP_ptr_->enable(MILLSTEP_5);
    // BP_ptr_->enablePointCloud();

    // mid360_ptr_->enable(MILLSTEP_5);
    // mid360_ptr_->enablePointCloud();

    // Field *mid360_tf = mid360_node_ptr_->getField("translation");
    // const double *mid360_pose = mid360_tf->getSFVec3f();
    // TODO:
    // mid360InitPose_[0] = mid360_pose[0];
    // mid360InitPose_[1] = mid360_pose[1];
    // mid360InitPose_[2] = mid360_pose[2];
}

void NormalSTController::enableIMU() {
    // get ptr
    inertial_unit_ptr_ = supervisor_->getInertialUnit("inertial unit");
    gyro_ptr_ = supervisor_->getGyro("gyro");
    accelerometer_ptr_ = supervisor_->getAccelerometer("accelerometer");

    // init
    inertial_unit_ptr_->enable(MILLSTEP_5);
    gyro_ptr_->enable(MILLSTEP_5);
    accelerometer_ptr_->enable(MILLSTEP_5);
}

void NormalSTController::getSteerWheelState() {
    // double pos_c = steer_pos_sensor_ptr_->getValue();
    // st_internal_vehicle_params_.rpm =
    //     (pos_c -
    //      st_internal_vehicle_params_.common_vehicle_params.steer_position) *
    //     SIM_FAC;
    // st_internal_vehicle_params_.common_vehicle_params.steer_position = pos_c;
    st_internal_vehicle_params_.common_vehicle_params.steer_position =
        steer_pos_sensor_ptr_->getValue();
}

void NormalSTController::setSteerWheelState() {
    // set steer yaw
    double steerYaw =
        st_internal_vehicle_params_.common_vehicle_params.steer_yaw;
    double steerSpeed =
        st_internal_vehicle_params_.common_vehicle_params.steer_speed;
    setWebotSteerWheel(steerYaw);
    // set steer speed
    double rotate_center[] = {0, 0, 0};
    auto pos_val = steer_pos_sensor_ptr_->getValue();
    int rotate_count = pos_val / (2 * PI);
    auto rotate_angle = pos_val - rotate_count * 2 * PI;
    double target_rotate[4] = {0, 0, 1, rotate_angle};
    steer_node_ptr_->getField("translation")->setSFVec3f(rotate_center);
    steer_node_ptr_->getField("rotation")->setSFRotation(target_rotate);
    steer_motor_ptr_->setVelocity(
        steerSpeed /
        supervisor_->getFromDef("S")->getField("radius")->getSFFloat());
    return;
}

void NormalSTController::setWebotSteerWheel(double yaw) {
    if (yaw > PI / 2)
        yaw = PI / 2;
    else if (yaw < -PI / 2)
        yaw = -PI / 2;
    Eigen::Vector3d origin_e(-PI / 2, 0, 0);
    Eigen::Vector3d new_e(-PI / 2, 0, 0 + yaw);

    Eigen::Quaterniond q;
    q = Euler2Quaternion(new_e);  ///按zyx轴的顺序旋转

    Eigen::AngleAxisd a(q);
    double new_aa[4] = {a.axis().x(), a.axis().y(), a.axis().z(), a.angle()};
    steer_node_ptr_->getField("rotation")->setSFRotation(new_aa);
}

void NormalSTController::setForkState() {
    fork_motor_ptr_->setVelocity(
        st_internal_vehicle_params_.common_vehicle_params.fork_speed);
}

void NormalSTController::getForkState() {
    // double local_fork_height = fork_pos_sensor_ptr_->getValue();
    // TODO: move this to SVC
    // if (std::abs(local_fork_height - FORK_LIFTUP_HEIGHT) <= HEIGHT_DEVIATION)
    // {
    //     st_internal_params_.fork_state = int(FORK_STATE::ON_FORK_TOP);
    // } else if (std::abs(local_fork_height - FORK_LIFTDOWN_HEIGHT) <=
    //            HEIGHT_DEVIATION) {
    //     st_internal_params_.fork_state = int(FORK_STATE::ON_FORK_BOTTOM);
    // } else {
    //     st_internal_params_.fork_state = int(FORK_STATE::ON_FORK_MIDDLE);
    // }
    st_internal_vehicle_params_.common_vehicle_params.fork_height =
        fork_pos_sensor_ptr_->getValue();
}

void NormalSTController::getIMUState() {
    const double *ptr_angle = gyro_ptr_->getValues();
    st_internal_vehicle_params_.imu.velocity[0] = ptr_angle[0];
    st_internal_vehicle_params_.imu.velocity[1] = ptr_angle[1];
    st_internal_vehicle_params_.imu.velocity[2] = ptr_angle[2];
    st_internal_vehicle_params_.imu.acceleration[0] =
        accelerometer_ptr_->getValues()[0];
    st_internal_vehicle_params_.imu.acceleration[1] =
        accelerometer_ptr_->getValues()[1];
    st_internal_vehicle_params_.imu.acceleration[2] =
        accelerometer_ptr_->getValues()[2];
}

void NormalSTController::setRobotState(
    const std::map<std::string, double> &msg) {
    if (isManual_) {
        st_internal_vehicle_params_.common_vehicle_params.steer_speed =
            msg.at("steer_speed");
        st_internal_vehicle_params_.common_vehicle_params.steer_yaw =
            msg.at("steer_yaw");
        st_internal_vehicle_params_.common_vehicle_params.fork_speed =
            msg.at("fork_speed");
    }
}

void NormalSTController::getRobotState(std::map<std::string, double> &msg) {
    msg["steer_speed"] =
        st_internal_vehicle_params_.common_vehicle_params.steer_speed;
    msg["steer_yaw"] =
        st_internal_vehicle_params_.common_vehicle_params.steer_yaw;
    msg["fork_speed"] =
        st_internal_vehicle_params_.common_vehicle_params.fork_speed;
    msg["fork_height"] =
        st_internal_vehicle_params_.common_vehicle_params.fork_height;
}

void NormalSTController::getVehicleYaw() {
    auto *robot_rot =
        supervisor_->getFromDef("RobotNode_ST")->getField("rotation");

    auto tmp_r = robot_rot->getSFRotation();

    Eigen::AngleAxisd tmp_angleaxis(
        tmp_r[3], Eigen::Vector3d(tmp_r[0], tmp_r[1], tmp_r[2]));
    Eigen::Vector3d r_eulerangle3 = tmp_angleaxis.matrix().eulerAngles(2, 1, 0);
    st_internal_vehicle_params_.common_vehicle_params.vehicle_yaw =
        r_eulerangle3[0];
}

void NormalSTController::init() {
    // IMU Report thread
    // std::thread imu_report_thread(
    //     std::bind(&NormalSTController::IMUReportSpin, this));
    // m_thread_["imu_report"] = std::move(imu_report_thread);
    // std::thread fork_report_thread(
    //     std::bind(&NormalSTController::ForkReportSpin, this));
    // m_thread_["fork_report"] = std::move(fork_report_thread);
    // std::thread bp_report_thread(
    //     std::bind(&NormalSTController::BpReportSpin, this));
    // m_thread_["bp_report"] = std::move(bp_report_thread);
    // bp_ptr_ = std::make_unique<VNSimLidar::BPLidar>(
    //     &webotsExited_, supervisor_, "BP", nullptr, "192.168.1.55");
    // std::thread local_thread(
    //     std::bind(&VNSimLidar::BPLidar::LidarSpin, bp_ptr.get()));
    // m_thread_["bp_report"] = std::move(local_thread);
}

void NormalSTController::whileSpin() {
    eCAL::protobuf::CPublisher<foxglove::ForkPose> pubFork("webot/forkpose");
    eCAL::protobuf::CPublisher<foxglove::Imu> pubIMU("webot/IMUData");
    eCAL::protobuf::CPublisher<foxglove::Vector2> pubWheelPosition(
        "webot/SteerWheelPosition");
    foxglove::ForkPose payload_forkPose;
    foxglove::Imu payload_imu;
    foxglove::Vector2 payload_wheelPosition;
    std::cout << 2 << std::endl;
    while (supervisor_->step(SIMULATION_STEP) != -1) {
        // READ
        getForkState();
        getSteerWheelState();
        getIMUState();
        getVehicleYaw();
        // TODO: check if subscriber is blocked
        // TODO: optimize this ...
        // REPORT
        payload_forkPose.set_z(
            st_internal_vehicle_params_.common_vehicle_params.fork_height);
        std::cout << 7.1 << std::endl;
        payload_imu.add_orientation_covariance(
            st_internal_vehicle_params_.common_vehicle_params.vehicle_yaw);
        std::cout << 7.2 << std::endl;
        for (int i = 0; i < 3; i++) {
            payload_imu.add_angular_velocity_covariance(
                st_internal_vehicle_params_.imu.velocity[i]);
            payload_imu.add_linear_acceleration_covariance(
                st_internal_vehicle_params_.imu.acceleration[i]);
        }
        payload_wheelPosition.set_x(
            st_internal_vehicle_params_.common_vehicle_params.steer_position);
        pubFork.Send(payload_forkPose);
        pubIMU.Send(payload_imu);
        pubWheelPosition.Send(payload_wheelPosition);
        // WRITE
        setSteerWheelState();
        setForkState();
    }
    webotsExited_ = true;
}

// void NormalSTController::BpReportSpin() {
// eCAL::protobuf::CPublisher<pb::PointCloud2> pubPts("192.168.1.55");
// auto pc_ptr = BP_ptr_->getPointCloud();
// auto layer_count = BP_ptr_->getNumberOfLayers();
// auto pt_count = BP_ptr_->getNumberOfPoints();
// auto all_pt_count = layer_count * pt_count;
// QElapsedTimer bpTimer;
// bpTimer.start();
// while (!webotsExited_) {
//     pb::PointCloud2 payload;
//     PointCloud2Init(payload, 28800);
//     char *pb_data_ptr = &((*payload.mutable_data())[0]);
//     for (int i = 0; i < all_pt_count; i++) {
//         if (std::abs(pc_ptr[i].x) != INFINITY &&
//             std::abs(pc_ptr[i].y) != INFINITY &&
//             std::abs(pc_ptr[i].z) != INFINITY) {
//             if (i < payload.width() - 1) {
//                 size_t pt_bt_i = i * PbPointBWidth;
//                 PointEncode(pc_ptr[i], pb_data_ptr + pt_bt_i);
//             } else {
//                 break;
//             }
//         }
//     }
//     payload.set_is_dense(false);
//     pubPts.Send(payload);
//     std::this_thread::sleep_for(
//         std::chrono::milliseconds(50 - bpTimer.elapsed()));
//     bpTimer.restart();
// }
// return;
//}
