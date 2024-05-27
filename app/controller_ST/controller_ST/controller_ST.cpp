#include <ecal/msg/protobuf/publisher.h>
// #include <qelapsedtimer.h>
#include <QtCore/QElapsedTimer>

#include <QDebug>
#include <QObject>
#include <QThread>
#include <thread>

#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "timestamp_generator.h"
#include "utils.hpp"
#include "keyboardform/keyboardform.h"
#include "controller_ST.h"
// #include "dataProcess.h"

#define SIMULATION_STEP 2  // 2ms??
//#define SIM_FAC 1.067061
#define SIM_FAC 1.06695119
#define SPEED_REAL_WORLD_TO_SIMULATION 35.0877193

#define WHEEL_STANDBY_HEIGHT 1.5
#define WHEEL_READY_HEIGHT 1.36
#define WHEEL_BOTTOM_HEIGHT 0.6
#define FORK_SPEED 0.1
#define FORK_LIFTUP_HEIGHT 0.085
#define FORK_LIFTDOWN_HEIGHT 0
#define WHEEL_STEP 0.02
#define HEIGHT_DEVIATION 0.0001

enum FORK_STATE { ON_FORK_BOTTOM = 0, ON_FORK_MIDDLE = 1, ON_FORK_TOP = 2 };

enum WHEEL_STATE { ON_WHEEL_STANDBY = 0, ON_WHEEL_READY = 1, ON_WHEEL_BOTTOM };

enum TOTAL_STATE { ON_BOTTOM = 0, ON_MIDDLE = 1, ON_TOP = 2 };

//////////////////////////////////////////////////////////////////////
/////////////////////////POINT CLOUD PB///////////////////////////////
struct PointFieldBw {
    std::string name;
    uint32_t bt_width;
    uint32_t datatype;
    uint32_t count;
};
constexpr size_t PbPointBWidth = 4 * 5 + 8;

static int test = 8;
static uint8_t temp = 0;

static uint8_t LL[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

inline void PointEncode(const webots::LidarPoint &pt, char *bit_array) {
    int intensity = 130;
    int label = 8;
    memcpy(bit_array, &(pt.x), 4);
    memcpy(bit_array + 4, &(pt.y), 4);
    memcpy(bit_array + 8, &(pt.z), 4);
    memcpy(bit_array + 12, &(intensity), 4);
    memcpy(bit_array + 16, &(label), 4);
    memcpy(bit_array + 20, &(pt.time), 8);
}

inline std::vector<PointFieldBw> CurrentPointFields() {
    return std::vector<PointFieldBw>{
        {"x", 4, 7, 1},         {"y", 4, 7, 1},     {"z", 4, 7, 1},
        {"intensity", 4, 7, 1}, {"label", 4, 6, 1}, {"timestamp", 8, 8, 1}};
}

void SetPointCloud22Fields(pb::PointCloud2 &pb) {
    pb.clear_fields();
    uint32_t offset = 0;
    auto pb_fields = CurrentPointFields();
    for (size_t i = 0; i < pb_fields.size(); i++) {
        auto field = pb.add_fields();
        field->set_name(pb_fields[i].name);
        field->set_datatype(pb_fields[i].datatype);
        if (i > 0)
            offset += pb_fields[i - 1].bt_width;
        field->set_offset(offset);
        field->set_count(pb_fields[i].count);
    }
    pb.set_is_bigendian(false);
    pb.set_point_step(PbPointBWidth);
    pb.set_is_dense(false);
    pb.set_height(1);
}

inline void NormalSTController::PointCloud2Init(pb::PointCloud2 &pb, int size) {
    static long long seq_ = 0;
    pb.mutable_header()->set_frame_id("");
    pb.mutable_header()->set_seq(seq_++);
    pb.mutable_header()->set_timestamp(generate_timestamp(lidar_count_));
    pb.set_height(1);
    pb.set_width(size);
    SetPointCloud22Fields(pb);
    pb.set_is_bigendian(false);
    size_t pt_bwidth = PbPointBWidth;
    pb.set_point_step(pt_bwidth);
    size_t pt_size = size;
    size_t pt_bt_size = pt_size * pt_bwidth;
    pb.set_row_step(pt_bt_size);
    pb.mutable_data()->resize(pt_bt_size);
}
////////////////////////////////////////////////////////////////////////////

NormalSTController::NormalSTController() {
    // TODO: This name consider to be in the config
    supervisor_ = new Supervisor();
    SteerWheelMotor_ = supervisor_->getMotor("FL");
    SteerWheelPositionSensor_ = SteerWheelMotor_->getPositionSensor();

    forkMotor_ = supervisor_->getMotor("fork height motor");
    forkPositionSensor_ = forkMotor_->getPositionSensor();

    steerNode_ = supervisor_->getFromDef("SteerSolid");

    mid360Node_ = supervisor_->getFromDef("MID360");
    mid360_ = supervisor_->getLidar("mid360");
    BP_ = supervisor_->getLidar("BP");

    inertialUnit_ = supervisor_->getInertialUnit("inertial unit");
    gyro_ = supervisor_->getGyro("gyro");
    accelerometer_ = supervisor_->getAccelerometer("accelerometer");
}

NormalSTController::~NormalSTController() {
    if (report_sencer_thread_.joinable()) {
        report_sencer_thread_.join();
    }
    if (report_mid360_thread_.joinable()) {
        report_mid360_thread_.join();
    }
    if (report_bp_thread_.joinable()) {
        report_bp_thread_.join();
    }
    if (supervisor_ != nullptr) {
        delete supervisor_;
    }
}

void NormalSTController::enableMotor() {
    SteerWheelMotor_->setPosition(INFINITY);
    SteerWheelMotor_->setVelocity(0);
    SteerWheelPositionSensor_->enable(MillStep_5);

    forkMotor_->setPosition(INFINITY);
    forkMotor_->setVelocity(0);
    forkPositionSensor_->enable(MillStep_5);
}

void NormalSTController::enableLidar3D() {
    BP_->enable(MillStep_5);
    BP_->enablePointCloud();

    mid360_->enable(MillStep_5);
    mid360_->enablePointCloud();

    Field *mid360_tf = mid360Node_->getField("translation");
    const double *mid360_pose = mid360_tf->getSFVec3f();
    mid360InitPose_[0] = mid360_pose[0];
    mid360InitPose_[1] = mid360_pose[1];
    mid360InitPose_[2] = mid360_pose[2];
}

void NormalSTController::enableIMU() {
    inertialUnit_->enable(MillStep_5);

    gyro_->enable(MillStep_5);

    accelerometer_->enable(MillStep_5);
}

void NormalSTController::updateVehicleYaw() {
    auto *robot_rot =
        supervisor_->getFromDef("RobotNode_ST")->getField("rotation");

    auto tmp_r = robot_rot->getSFRotation();

    Eigen::AngleAxisd tmp_angleaxis(
        tmp_r[3], Eigen::Vector3d(tmp_r[0], tmp_r[1], tmp_r[2]));
    Eigen::Vector3d r_eulerangle3 = tmp_angleaxis.matrix().eulerAngles(2, 1, 0);
    vehicleYaw_ = r_eulerangle3[0];
}

void NormalSTController::setSteerWheelYaw(double yaw) {
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

    // TODO: remove this
    static Supervisor *_super = Supervisor::getSupervisorInstance();
    static Node *steerWheel = _super->getFromDef("SteerWheel");
    static Field *steerWheelField = steerWheel->getField("rotation");
    steerWheelField->setSFRotation(new_aa);
}

void NormalSTController::writeFork() {
    forkMotor_->setVelocity(forkSpeed_);
}

void NormalSTController::writeSteerWheel() {
    // set steer yaw
    double steerYaw = steerYaw_;
    double steerSpeed = steerSpeed_;
    setSteerWheelYaw(steerYaw);
    // set steer speed
    double rotate_center[] = {0, 0, 0};
    auto pos_val = SteerWheelPositionSensor_->getValue();
    int rotate_count = pos_val / (2 * PI);
    auto rotate_angle = pos_val - rotate_count * 2 * PI;
    double target_rotate[4] = {0, 0, 1, rotate_angle};
    steerNode_->getField("translation")->setSFVec3f(rotate_center);
    steerNode_->getField("rotation")->setSFRotation(target_rotate);
    SteerWheelMotor_->setVelocity(
        steerSpeed /
        supervisor_->getFromDef("S")->getField("radius")->getSFFloat());
    return;
}

int NormalSTController::init() {
    enableMotor();
    enableLidar3D();
    enableIMU();
    updateForkState();
    pos_ = SteerWheelPositionSensor_->getValue();
    return 0;
}

void NormalSTController::updateRPM() {
    double pos_c = SteerWheelPositionSensor_->getValue();
    RPM_ = (pos_c - pos_) * SIM_FAC;
    pos_ = pos_c;
}

void NormalSTController::onRemoteSpeedChange(double steerSpeed, double steerYaw,
                                             double forkSpeed) {
    if (!isManual_) {
        steerSpeed_ = steerSpeed;
        steerYaw_ = steerYaw;
        forkSpeed_ = forkSpeed;
    }
}

void NormalSTController::onManualSpeedChange(double steerSpeed, double steerYaw,
                                             double forkSpeed,
                                             double pushUpHeight) {
    if (isManual_) {
        steerSpeed_ = steerSpeed * 0.1;  // slow down to get value
        steerYaw_ = steerYaw;
        forkSpeed_ = forkSpeed;
    }
}

void NormalSTController::updateInertialUnit() {
    const double *ptr_angle = gyro_->getValues();
    inertial_angular_velocity_[0] = ptr_angle[0];
    inertial_angular_velocity_[1] = ptr_angle[1];
    inertial_angular_velocity_[2] = ptr_angle[2];
    // std::cout << "row_v = " << inertial_angular_velocity_[0] << ", pitch_v =
    // "
    // << inertial_angular_velocity_[1] << ", yaw_v = " <<
    // inertial_angular_velocity_[2] << std::endl;
    acc_meter_value_[0] = accelerometer_->getValues()[0];
    acc_meter_value_[1] = accelerometer_->getValues()[1];
    acc_meter_value_[2] = accelerometer_->getValues()[2];
    // std::cout << "ax = " << acc_meter_value_[0] << ", ay = " <<
    // acc_meter_value_[1] << ", az = " << acc_meter_value_[2] << std::endl;
}

void NormalSTController::updateForkState() {
    double local_fork_height = forkPositionSensor_->getValue();
    if (std::abs(local_fork_height - FORK_LIFTUP_HEIGHT) <= HEIGHT_DEVIATION) {
        fork_state_ = int(FORK_STATE::ON_FORK_TOP);
    } else if (std::abs(local_fork_height - FORK_LIFTDOWN_HEIGHT) <=
               HEIGHT_DEVIATION) {
        fork_state_ = int(FORK_STATE::ON_FORK_BOTTOM);
    } else {
        fork_state_ = int(FORK_STATE::ON_FORK_MIDDLE);
    }
    if ((fork_state_ == int(FORK_STATE::ON_FORK_TOP))) {
        total_state_ = int(TOTAL_STATE::ON_TOP);
    } else if ((fork_state_ == int(FORK_STATE::ON_FORK_BOTTOM))) {
        total_state_ = int(TOTAL_STATE::ON_BOTTOM);
    } else {
        total_state_ = int(TOTAL_STATE::ON_MIDDLE);
    }
    // std::cout << "update --> [" << fork_state_ << " " << wheel_state_ << " "
    // << total_state_ << "]" <<std::endl;
}

bool NormalSTController::_readDeltaXYZ() {
    static auto pos_s =
        supervisor_->getFromDef("FJJ")->getField("position")->getSFFloat();
    auto t_robot_tran = supervisor_->getFromDef("RobotNode_ST")
                            ->getField("translation")
                            ->getSFVec3f();
    static double o_tran[3] = {t_robot_tran[0], t_robot_tran[1],
                               t_robot_tran[2]};

    // std::cout << "delta_x = " << o_tran[0] - t_robot_tran[0] << " " <<
    // std::abs(o_tran[0] - t_robot_tran[0]) << std::endl;
    if (std::abs(o_tran[0] - t_robot_tran[0]) > 30) {
        std::cout << "Yeah delta_x = " << o_tran[0] - t_robot_tran[0] << " "
                  << std::abs(o_tran[0] - t_robot_tran[0]) << ", deltaPos = "
                  << (supervisor_->getFromDef("FJJ")
                          ->getField("position")
                          ->getSFFloat() -
                      pos_s)
                  << std::endl;
        return true;
    }
    return false;
}

bool OnStop = false;

void NormalSTController::runSimulation() {
    // init
    // std::thread local_report_sencer([&]() {
    //   uint8_t switchValue[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    //                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //   std::vector<struct UpdateValue *> SwitchSencerInput;
    //   struct UpdateValue switchValueList[12];
    //   SwitchSencerInput.resize(12);
    //   auto last_check = std::chrono::system_clock::now();
    //   while (!webotsExited_) {
    //     // if (OnStop) {
    //     //   continue;
    //     // }
    //     auto now_t = std::chrono::system_clock::now();
    //     auto duration_t =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(
    //         now_t - last_check);
    //     if (duration_t.count() >= 10) {
    //       last_check = std::chrono::system_clock::now();

    //       {
    //         // QElapsedTimer test_timer;
    //         // auto incrementalCoderVal = sencerSet[0]->solveVal(_steerYaw);
    //         // auto gyroscope_val       =
    //         sencerSet[2]->solveVal(_vehicleYaw);
    //         // TODO: we may be need a lock to do the data syncronization
    //         // qDebug() << __FUNCTION__ << " --> local_report_sencer";
    //         uint16_t battery_device = 100;
    //         uint32_t d_upload = dataidx_upload_++;
    //         struct UpdateValue battery_sencer_value = {&battery_device,
    //                                                    sizeof(uint16_t)};
    //         struct UpdateValue dataIdx_value = {&dataIdx_, sizeof(uint32_t)};
    //         struct UpdateValue dataIdx_upload_value = {&d_upload,
    //                                                    sizeof(uint32_t)};
    //         std::vector<double> incrementalCoder_cal_input;
    //         std::vector<double> GyroScope_cal_input;
    //         // std::vector<double> WheelCoder_cal_input;
    //         // std::vector<double> forkPositionSensor_ptcal_input;
    //         std::vector<double> AccelerometerXInput;
    //         std::vector<double> AccelerometerYInput;
    //         std::vector<double> AccelerometerZInput;
    //         std::vector<double> AngularVelocitySensorXInput;
    //         std::vector<double> AngularVelocitySensorYInput;
    //         std::vector<double> AngularVelocitySensorZInput;
    //         std::vector<double> RPMInput;
    //         std::vector<double> fake_cal_input;
    //         std::vector<struct UpdateValue *> fake_direct_input;
    //         std::vector<struct UpdateValue *> battery_sencer_input;
    //         std::vector<struct UpdateValue *> dataIdx_input;
    //         std::vector<struct UpdateValue *> dataIdx_upload_input;
    //         incrementalCoder_cal_input.push_back(steerYaw_);
    //         // std::cout << "send steerYaw_ = " << steerYaw_ << std::endl;
    //         GyroScope_cal_input.push_back(vehicleYaw_);
    //         RPMInput.push_back(RPM_);
    //         // std::cout << inertial_angular_velocity_[0] << " " <<
    //         // inertial_angular_velocity_[1] << " " <<
    //         // inertial_angular_velocity_[2] << std::endl;
    //         AccelerometerXInput.push_back(acc_meter_value_[0]);  // X
    //         AccelerometerYInput.push_back(acc_meter_value_[1]);  // Y
    //         AccelerometerZInput.push_back(acc_meter_value_[2]);  // Z
    //         AngularVelocitySensorXInput.push_back(
    //             inertial_angular_velocity_[0]);  // roll
    //         AngularVelocitySensorYInput.push_back(
    //             inertial_angular_velocity_[1]);  // pitch
    //         AngularVelocitySensorZInput.push_back(
    //             inertial_angular_velocity_[2]);  // yaw
    //         battery_sencer_input.push_back(&battery_sencer_value);
    //         dataIdx_input.push_back(&dataIdx_value);
    //         dataIdx_upload_input.push_back(&dataIdx_upload_value);
    //         // std::cout << "total_state_ --> " << total_state_ << std::endl;
    //         if (total_state_ == int(TOTAL_STATE::ON_TOP)) {
    //           switchValue[4] = 0x01;
    //         } else if (total_state_ == int(TOTAL_STATE::ON_BOTTOM)) {
    //           switchValue[4] = 0x02;
    //         } else {
    //           switchValue[4] = 0x00;
    //         }
    //         // printf("total_state = %d switch [%02X %02X]\n", total_state_,
    //         // switchValue[3], switchValue[4]);
    //         // int index = test /8 - 1;
    //         // int shift = test%8;
    //         // std::cout << "index = " << index << ", shift = " << shift <<
    //         // std::endl; switchValue[index] = LL[shift];
    //         for (int i = 0; i < 12; i++) {
    //           // if(i != index)
    //           //     switchValue[i] = 0x00;
    //           switchValueList[i].val = &switchValue[i];
    //           switchValueList[i].len = 1;
    //           switchValueList[i].subId = i;
    //           SwitchSencerInput[i] = &switchValueList[i];
    //         }
    //         encoder_->updateValue("IncrementalSteeringCoder",
    //                               incrementalCoder_cal_input,
    //                               fake_direct_input);
    //         encoder_->updateValue("Gyroscope", GyroScope_cal_input,
    //                               fake_direct_input);
    //         // encoder_->updateValue("WheelCoder", WheelCoder_cal_input,
    //         // fake_direct_input); encoder_->updateValue("HeightCoder",
    //         // forkPositionSensor_ptcal_input, fake_direct_input);
    //         // encoder_->updateValue("ForkDisplacementSencerZ",
    //         // forkPositionSensor_ptcal_input, fake_direct_input);
    //         encoder_->updateValue("RPMSensor", RPMInput, fake_direct_input);
    //         encoder_->updateValue("BatterySencer", fake_cal_input,
    //                               battery_sencer_input);
    //         encoder_->updateValue("DataIndex", fake_cal_input,
    //                               dataIdx_upload_input);
    //         encoder_->updateValue("DataIndexReturn", fake_cal_input,
    //                               dataIdx_input);
    //         encoder_->updateValue("SwitchSencer", fake_cal_input,
    //                               SwitchSencerInput);
    //         encoder_->updateValue("AccelerometerX", AccelerometerXInput,
    //                               fake_direct_input);
    //         encoder_->updateValue("AccelerometerY", AccelerometerYInput,
    //                               fake_direct_input);
    //         encoder_->updateValue("AccelerometerZ", AccelerometerZInput,
    //                               fake_direct_input);
    //         encoder_->updateValue("AngularVelocitySensorX",
    //                               AngularVelocitySensorXInput,
    //                               fake_direct_input);  // roll
    //         encoder_->updateValue("AngularVelocitySensorY",
    //                               AngularVelocitySensorYInput,
    //                               fake_direct_input);  // pitch
    //         encoder_->updateValue("AngularVelocitySensorZ",
    //                               AngularVelocitySensorZInput,
    //                               fake_direct_input);  // yaw
    //         // static uint32_t data_count = 0;
    //         // static std::stringstream ss_str;

    //         // auto sensor_str =
    //         //
    //         parseSensorData(mWheelCoder.wheel_l_code,mWheelCoder.wheel_r_code,incrementalCoderVal,_forkHeight,gyroscope_val);
    //         lidar_count_++;  //避免uin32_t长度不够
    //         const struct Package *pack = encoder_->encodePackage();
    //         sensorPub((char *) pack->buf, pack->len);
    //         // std::ostringstream oss;
    //         // for(int i = 0; i < pack->len; i++){
    //         //     oss << std::hex << static_cast<unsigned>(pack->buf[i]) <<
    //         "
    //         ";
    //         // }
    //         // std::string msg = oss.str();
    //         // std::string cmd = "echo " + msg + " >> Xuyang.log";
    //         // system(cmd.c_str());
    //         // printf("len = %d, [", pack->len);
    //         // for(int i = 0; i < pack->len; i++){
    //         //     printf("%.2X ", pack->buf[i]);
    //         // }
    //         // printf("]\n");
    //       }
    //     } else {
    //       // keep reporting rate at 100HZ idealy
    //       std::this_thread::sleep_for(
    //           std::chrono::milliseconds(10 - duration_t.count()));
    //     }
    //   }
    // });
    std::thread local_bp_thread([&]() {
        // eCAL::protobuf::CPublisher<pb::PointCloud2> pubPts("multi_mid360");
        // TODO: consider change this ip
        eCAL::protobuf::CPublisher<pb::PointCloud2> pubPts("192.168.1.55");
        auto pc_ptr = BP_->getPointCloud();
        auto layer_count = BP_->getNumberOfLayers();
        auto pt_count = BP_->getNumberOfPoints();
        auto all_pt_count = layer_count * pt_count;
        QElapsedTimer bpTimer;
        bpTimer.start();
        while (!webotsExited_) {
            pb::PointCloud2 payload;
            PointCloud2Init(payload, 28800);
            char *pb_data_ptr = &((*payload.mutable_data())[0]);
            for (int i = 0; i < all_pt_count; i++) {
                if (std::abs(pc_ptr[i].x) != INFINITY &&
                    std::abs(pc_ptr[i].y) != INFINITY &&
                    std::abs(pc_ptr[i].z) != INFINITY) {
                    if (i < payload.width() - 1) {
                        size_t pt_bt_i = i * PbPointBWidth;
                        PointEncode(pc_ptr[i], pb_data_ptr + pt_bt_i);
                    } else {
                        break;
                    }
                }
            }
            payload.set_is_dense(false);
            pubPts.Send(payload);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(50 - bpTimer.elapsed()));
            bpTimer.restart();
        }
    });
    // TODO: LoadMid360 Data
    // std::thread local_mid360_report([&]() {
    //     // eCAL::protobuf::CPublisher<pb::PointCloud2>
    //     pubPts("multi_mid360"); eCAL::protobuf::CPublisher<pb::PointCloud2>
    //     pubPts("mid360pub");
    //     // Lidar *mid360= _super->getLidar("mid360");
    //     auto pc_ptr = mid360_->getPointCloud();
    //     auto layer_count = mid360_->getNumberOfLayers();
    //     auto pt_count = mid360_->getNumberOfPoints();

    //     auto mid_fb = loadMid360FB();
    //     auto &mid_idx_itr = mid_fb["mid360"].simDataIndex;

    //     std::vector<const LidarPoint *> stash_pt;
    //     for (int i = 0; i < layer_count; i++) {
    //         const LidarPoint *pc = mid360_->getLayerPointCloud(i);
    //         stash_pt.push_back(pc);
    //     }

    //     if (mid_idx_itr.empty()) {
    //         qDebug() << "mid fb is empty";
    //         return;
    //     }

    //     auto last_check = std::chrono::system_clock::now();

    //     QElapsedTimer test_time;
    //     test_time.start();
    //     while (!webotsExited_) {
    //         auto now_t = std::chrono::system_clock::now();
    //         auto duration_t =
    //             std::chrono::duration_cast<std::chrono::milliseconds>(
    //                 now_t - last_check);

    //         if (duration_t.count() < 100) {
    //             std::this_thread::sleep_for(
    //                 std::chrono::milliseconds(100 - duration_t.count()));
    //         }

    //         QElapsedTimer point_time;
    //         point_time.start();
    //         pb::PointCloud2 payload;
    //         PointCloud2Init(payload, 20722);

    //         char *pb_data_ptr = &((*payload.mutable_data())[0]);
    //         int step_byte = 0;
    //         int spin_count = 0;
    //         for (const auto &_t : mid_idx_itr) {
    //             int pc_idx = _t.pc_idx;
    //             int layer_id = _t.layer_count;
    //             if (layer_id >= layer_count || pc_idx >= pt_count)
    //                 continue;

    //             auto pc = stash_pt[layer_id];
    //             if (std::abs(pc[pc_idx].x) != INFINITY &&
    //                 std::abs(pc[pc_idx].y) != INFINITY &&
    //                 std::abs(pc[pc_idx].z) != INFINITY) {
    //                 if (++step_byte < 20721) {
    //                     size_t pt_bt_i = step_byte * PbPointBWidth;

    //                     PointEncode(pc[pc_idx], pb_data_ptr + pt_bt_i);
    //                 } else {
    //                     break;
    //                 }
    //             }
    //         }
    //         payload.set_is_dense(false);
    //         pubPts.Send(payload);

    //         last_check = std::chrono::system_clock::now();
    //         test_time.restart();
    //     }
    // });

    // report_sencer_thread_ = std::move(local_report_sencer); // TODO: send
    // standard ecal data to svc_model_ST
    report_bp_thread_ = std::move(local_bp_thread);
    // report_mid360_thread_ = std::move(local_mid360_report);

    while (supervisor_->step(SIMULATION_STEP) != -1) {
        // test distance update
        // if (_readDeltaXYZ() == true) {
        //   if (!OnStop) {
        //     std::cout << "stop distance occur!" << std::endl;
        //     // SteerWheelMotor_->setVelocity(0);
        //     // BL1_->setVelocity(0);
        //     // BR1_->setVelocity(0);
        //   }
        //   OnStop = true;
        //   while (true) { sleep(1000); }
        //   continue;
        // }
        // READ
        updateForkState();
        updateRPM();
        updateVehicleYaw();
        updateInertialUnit();
        // WRITE
        writeSteerWheel();
        writeFork();
    }
    webotsExited_ = true;
}

void NormalSTController::onRemoteMsg(uint8_t *msg, int len) {
    // struct Package pack {
    //     msg, len
    // };
    uint32_t DataIndex = 0;
    double fakeDataIndex = 0;
    int16_t MoveDevice = 0;
    double realMoveDevice = 0;
    int16_t SteeringDevice = 0;
    double realSteeringDevice = 0;
    int16_t ForkDeviceZ = 0;
    double realForkDeviceZ = 0.02;
    double nothing = 0;
    uint8_t switch_sensor[10] = {0xFF};
    // TODO: svc_model...
    // decoder_->decodePackage(&pack);
    // decoder_->getValue("DataIndex", &DataIndex, &fakeDataIndex);
    // decoder_->getValue("MoveDevice", &MoveDevice, &realMoveDevice);
    // decoder_->getValue("SteeringDevice", &SteeringDevice,
    // &realSteeringDevice); decoder_->getValue("ForkDeviceZ", &ForkDeviceZ,
    // &realForkDeviceZ); decoder_->getValue("SwitchActuator", &switch_sensor,
    // &nothing); bool LiftUp = (switch_sensor[5] >> 7) & 0x01;    // 47 bool
    // LiftDown = (switch_sensor[6] >> 0) & 0x01;  // 48
    dataIdx_ = DataIndex;
    onRemoteSpeedChange(realMoveDevice, realSteeringDevice, realForkDeviceZ);
    //_dealWheelAndFork(0, LiftUp, LiftDown); future use may be
    // printf("len = %d, [", len);
    // for(int i = 0; i < len; i++){
    //     printf("%.2X ", msg[i]);
    // }
    // printf("]\n");
    // std::cout << "DataIndex = " << DataIndex << ", MoveDevice = " <<
    // realMoveDevice << ", SteeringDevice = " << realSteeringDevice << ",
    // LiftUp
    // = " << LiftUp \
    //           << ", LiftDown = " << LiftDown << ", forkZ = " <<
    //           realForkDeviceZ
    //           << std::endl;
}

void NormalSTController::onManualMsg(const char *msg) {
    KeyboardForm::decodeMsg(msg, keyboard_input_);
    onManualSpeedChange(keyboard_input_["steerSpeed"],
                        keyboard_input_["steerYaw"],
                        keyboard_input_["forkSpeed"]);
}

void NormalSTController::onManualReport(std::string &msg) {
    keyboard_output_["steerSpeed"] = steerSpeed_;
    keyboard_output_["steerYaw"] = steerYaw_;
    keyboard_output_["forkSpeed"] = forkSpeed_;
    keyboard_output_["forkHeight"] = forkPositionSensor_val_;
    keyboard_output_["realSpeed"] = 0;
    KeyboardForm::encodeMsg(keyboard_output_, msg);
}
