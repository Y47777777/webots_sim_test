#include <ecal/msg/protobuf/publisher.h>
#include "geometry/geometry.h"
#include <qelapsedtimer.h>

// #include "foxglove-vn/Imu.pb.h"       // IMU
// #include "foxglove-vn/ForkPose.pb.h"  // vehicle height
// #include "foxglove-vn/Vector2.pb.h"   // use for origin steer wheel position
#include "controller_ST.h"
#include <QElapsedTimer>
#include <QTime>

using namespace VNSim;
using namespace webots;

#define SIMULATION_STEP 2
//#define SERIAL_MSG_BUF 256
#define BP_LIDAR_MSG_BUF 900000
// #define FORK_LIFTUP_HEIGHT 0.085  // TODO: it should be config
// #define FORK_LIFTDOWN_HEIGHT 0    // TODO: it should be config
// #define HEIGHT_DEVIATION 0.0001   // meter
// #define SIM_FAC 1.06695119

enum FORK_STATE { ON_FORK_BOTTOM = 0, ON_FORK_MIDDLE = 1, ON_FORK_TOP = 2 };

struct PointFieldBw {
    std::string name;
    uint32_t bt_width;
    uint32_t datatype;
    uint32_t count;
};

constexpr size_t PbPointBWidth = 4 * 5 + 8;

///////////////////////////////////////////TEST CODE REMOVE
/// ME///////////////////////////////////////
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
    pb.mutable_header()->set_timestamp(0);
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
////////////////////////////////////////////////////////////////////////////////////////

NormalSTController::NormalSTController() : BaseController() {
    // sensor init
    BP_ptr_ = std::make_shared<WLidar>("BP");
    mid360_ptr_ = std::make_shared<WLidar>("mid360", "MID360", 100);
    imu_ptr_ = std::make_shared<WImu>("inertial unit", "gyro", "accelerometer");
    // motor init
    fork_ptr_ = std::make_shared<WFork>("fork height motor");
    stree_ptr_ =
        std::make_shared<WWheel>("FL", "SteerWheel", "SteerSolid", "S");

    // TODO: creat task
    v_while_spin_.push_back(bind(&WBase::spin, stree_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, fork_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, imu_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, BP_ptr_));
    // v_while_spin_.push_back(bind(&WBase::spin, mid360_ptr_));

    std::thread local_thread(
        std::bind(&NormalSTController::BpReportSpin, this));
    m_thread_["bp_report"] = std::move(local_thread);

    ecal_wrapper_.init(true, "webots_ST");
    ecal_wrapper_.addEcal("webot/ST_msg");
    ecal_wrapper_.addEcal("webot/pointCloud");
    ecal_wrapper_.addEcal(
        "svc_model_st/ST_msg",
        std::bind(&NormalSTController::onRemoteSerialMsg, this,
                  std::placeholders::_1, std::placeholders::_2));

        payload_Up.set_allocated_imu(&payload_imu);
        payload.set_allocated_up_msg(&payload_Up);
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
    msg["steer_speed"] = stree_ptr_->getSpeed();
    msg["steer_yaw"] = stree_ptr_->getMotorYaw();
    msg["fork_speed"] = fork_ptr_->getVelocityValue();
    msg["fork_height"] = fork_ptr_->getSenosorValue();
    msg["real_speed"] = 0;
    // TODO: fork_speed real_speed
}

void NormalSTController::whileSpin() {
    // 主循环 在super_->step()后
    this->sendSerialSpin();
}

void NormalSTController::onRemoteSerialMsg(
    const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    if (!isManual_) {
        sim_data_flow::STMsg payload;
        payload.ParseFromArray(data->buf, data->size);
        stree_ptr_->setSpeed(payload.down_msg().steering_speed(),
                             payload.down_msg().steering_theta());
        fork_ptr_->setVelocity(payload.down_msg().forkspeedz());
    }
}

void NormalSTController::sendSerialSpin() {
    payload_Up.set_forkposez(fork_ptr_->getSenosorValue());
    payload_Up.set_steerposition(stree_ptr_->getSenosorValue());
    payload_imu.add_orientation_covariance(imu_ptr_->getVehicleYaw());  //
    // z
    for (int i = 0; i < 3; i++) {
        payload_imu.add_angular_velocity_covariance(imu_ptr_->getGyroValue(i));
        payload_imu.add_linear_acceleration_covariance(
            imu_ptr_->getAccValue(i));
    }
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_wrapper_.send("webot/ST_msg", buf, payload.ByteSize());
    payload_imu.Clear();
}

void NormalSTController::BpReportSpin() {
    eCAL::protobuf::CPublisher<pb::PointCloud2> pubPts("webot/pointCloud");
    uint8_t buf[BP_LIDAR_MSG_BUF];
    LOG_INFO("BpReportSpin start\n");
    // TODO: solve this
    // std::vector<const LidarPoint *> points;
    // BP_ptr_->getPointCloudPtr(points);
    auto lidar_ptr = BP_ptr_->getLidarPtr();
    auto pc_ptr = lidar_ptr->getPointCloud();
    auto layer_count = lidar_ptr->getNumberOfLayers();
    auto pt_count = lidar_ptr->getNumberOfPoints();
    auto all_pt_count = layer_count * pt_count;
    if (all_pt_count == 0) {
        // LOG_ERROR("BpReportSpin --> empty lidar list\n");
        return;
    }
    // auto pc_ptr = lidar_list[0]->getPointCloud();
    // auto layer_count = lidar_list[0]->getNumberOfLayers();
    // auto pt_count = lidar_list[0]->getNumberOfPoints();
    // auto all_pt_count = layer_count * pt_count;
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
        if (BP_LIDAR_MSG_BUF < payload.ByteSize()) {
            LOG_WARN(
                "byte size not good, ignore sending..., point cloud "
                "size = %d",
                payload.ByteSize());
            std::this_thread::sleep_for(
                std::chrono::milliseconds(50 - bpTimer.elapsed()));
            bpTimer.restart();
            continue;
        }
        payload.SerializePartialToArray(buf, payload.ByteSize());
        ecal_wrapper_.send("webot/pointCloud", buf, payload.ByteSize());
        std::this_thread::sleep_for(
            std::chrono::milliseconds(50 - bpTimer.elapsed()));
        bpTimer.restart();
    }
    return;
}
