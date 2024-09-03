/**
 * @file w_barcodescanner.h
 * @author weijchen weijchen@visionnav.com
 * @brief
 * @version 0.1
 * @date 2024-08-03
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <map>
#include <webots/Node.hpp>
#include <webots/Lidar.hpp>
#include "geometry/geometry.h"
#include "webots_device/w_base.h"
#include "logvn/logvn.h"
#include "code_scan.h"

namespace VNSim {
using namespace webots;
class WBarcode {
   public:
    WBarcode(Node *this_ptr) {
        this_ptr_ = this_ptr;
        name = this_ptr_->getField("name")->getSFString();
        copyFrom(this_ptr_->getPose());
        QRCode = this_ptr_->getField("QRcode")->getSFString();
        memcpy(size_, this_ptr_->getField("size")->getSFVec3f(),
               sizeof(double) * 3);

        double size_x = size_[0] + 0.03;
        double size_y = size_[1] + 0.03;
        double size_z = size_[2] + 0.03;

        // 八个顶点 !顶点需要按照顺序
        point_list.clear();
        point_list.push_back(
            Eigen::Vector4d(size_x / 2, size_y / 2, size_z / 2, 1));
        point_list.push_back(
            Eigen::Vector4d(-size_x / 2, size_y / 2, size_z / 2, 1));
        point_list.push_back(
            Eigen::Vector4d(-size_x / 2, -size_y / 2, size_z / 2, 1));
        point_list.push_back(
            Eigen::Vector4d(size_x / 2, -size_y / 2, size_z / 2, 1));

        point_list.push_back(
            Eigen::Vector4d(size_x / 2, -size_y / 2, -size_z / 2, 1));
        point_list.push_back(
            Eigen::Vector4d(-size_x / 2, -size_y / 2, -size_z / 2, 1));
        point_list.push_back(
            Eigen::Vector4d(-size_x / 2, size_y / 2, -size_z / 2, 1));
        point_list.push_back(
            Eigen::Vector4d(size_x / 2, size_y / 2, -size_z / 2, 1));

        calculateBoundingBox();

        LOG_INFO("creat Barcode %s", name.c_str());
    }

    void transferCheck() {
        auto pose_itr = this_ptr_->getPose();
        if (fabs(pose_itr[3] - tran_matrix(0, 3)) > 0.01 ||
            fabs(pose_itr[7] - tran_matrix(1, 3)) > 0.01 ||
            fabs(pose_itr[11] - tran_matrix(2, 3)) > 0.01) {
            // xyz发生变化
            copyFrom(pose_itr);
            calculateBoundingBox();
        }
    }

    bool checkBeScanned(const Eigen::Vector4d &point) {
        // 先通过 z 排除
        if (point.z() < min_z)
            return false;
        if (point.z() > max_z)
            return false;

        // 通过中心位置排除 > 1m
        if (fabs(point.x() - center.x()) > 1)
            return false;
        if (fabs(point.y() - center.y()) > 1)
            return false;

        // LOG_INFO("min_z %.2f", min_z);
        // LOG_INFO("max_z %.2f", max_z);
        // LOG_INFO("point.z %.2f", point.z());
        if (isPonitInCube(point)) {
            return true;
        }

        return false;
    }

    const char *getQRcode() { return QRCode.c_str(); }

   private:
    void copyFrom(const double *pose_itr) {
        double pose[16];
        memcpy(pose, pose_itr, 16 * sizeof(double));
        tran_matrix =
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(pose);
    }

    void calculateBoundingBox() {
        // 变换顶点至 world_link
        min_z = MAXFLOAT;
        max_z = -MAXFLOAT;
        std::vector<Eigen::Vector4d> p_list;
        for (int i = 0; i < point_list.size(); i++) {
            p_list.push_back(tran_matrix * point_list[i]);
            min_z = std::min(min_z, p_list[i].z());
            max_z = std::max(max_z, p_list[i].z());
        }
        // 创建中心
        center = tran_matrix.col(3);
        LOG_INFO("center: %.2f, %.2f, %.2f", center.x(), center.y(), center.z(),
                 center.w());

        // 计算包围盒的轴
        min_list.clear();
        max_list.clear();
        axis_list.clear();
        double minDotCube = MAXFLOAT;
        double maxDotCube = -MAXFLOAT;

        for (int i = 0; i < 12; i++) {
            Eigen::Vector4d axis;
            // 计算轴（立方体的边）
            if (i < 4) {
                axis = p_list[i + 1] - p_list[i];
            } else if (i < 8) {
                axis = p_list[i - 3] - p_list[i - 4];
            } else {
                axis = p_list[i - 8] - p_list[i - 7];
            }
            axis_list.push_back(axis);

            minDotCube = dotProduct(p_list[0], axis);
            maxDotCube = minDotCube;
            for (int j = 1; j < 8; j++) {
                double dotCube = dotProduct(p_list[j], axis);
                minDotCube = std::min(minDotCube, dotCube);
                maxDotCube = std::max(maxDotCube, dotCube);
            }
            min_list.push_back(minDotCube);
            max_list.push_back(maxDotCube);
        }
    }

    bool isPonitInCube(const Eigen::Vector4d &point) {
        for (int i = 0; i < axis_list.size(); i++) {
            double dotP = dotProduct(point, axis_list[i]);
            if (dotP < min_list[i] || dotP > max_list[i]) {
                return false;
            }
        }
        return true;
    }

   private:
    Node *this_ptr_ = nullptr;
    Eigen::Matrix4d tran_matrix;  // 世界位姿
    std::string QRCode = "";
    std::string name = "";

    // 反光板判斷
    double size_[3] = {0};
    std::vector<Eigen::Vector4d> point_list;
    std::vector<Eigen::Vector4d> axis_list;
    std::vector<double> min_list;
    std::vector<double> max_list;
    Eigen::Vector4d center;
    double min_z = MAXFLOAT;
    double max_z = -MAXFLOAT;
};

class WBarcodeScan : public WBase {
   public:
    /**
     * @brief Construct a new WBarcodeScan object
     *
     */
    WBarcodeScan(std::shared_ptr<CoderNotifyer> notifyer,
                 std::string scaner_name, int frequency = 500,
                 int success_size = 10) {
        notifyer_ = notifyer;
        name_ = scaner_name;
        enable_ = false;
        // creat Barcode, only once
        if (v_barcode_.empty()) {
            super_->step(step_duration_);
            getChildNode(super_->getRoot());
        }

        // creat scaner
        lidar_ = super_->getLidar(scaner_name);
        if (lidar_ == nullptr) {
            LOG_ERROR("%s is nullptr", scaner_name.c_str());
            return;
        }
        frequency_ = frequency;
        frequency_cnt_ = std::round(double(frequency_ / step_duration_));
        lidar_->enable(frequency_);
        lidar_->enablePointCloud();

        start_point_ = 0;
        end_point_ = lidar_->getNumberOfPoints();

        this_node_ = super_->getFromDevice(lidar_);

        success_size_ = success_size;

        // finsih
        cur_step_ = super_->getStepCnt();
        start_step_ = super_->getStepCnt();
        LOG_INFO("creat scnaner %s", scaner_name.c_str());
    }

    ~WBarcodeScan() {}

    const char *getQRCode() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        // TODO: 获取码值
        // return "1";
        if (target_barcode_ != nullptr) {
            return target_barcode_->getQRcode();
        }
        return "";
    }

    void scanEnableSet(bool enable) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        enable_ = enable;
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        if (enable_ == false) {
            return;
        }

        int cur_step_cnt = super_->getStepCnt() - start_step_;
        if (cur_step_cnt % frequency_cnt_ != 0) {
            return;
        }

        // 检查barcode位姿是否发生变化 ecah step only check once
        if (cur_step_ != super_->getStepCnt()) {
            cur_step_ = super_->getStepCnt();
            for (auto &barcode : v_barcode_) {
                // TODO: 用车体坐标加速
                barcode->transferCheck();
            }
        }

        // 判断激光点

        const LidarPoint *address = lidar_->getPointCloud();

        double pose[16];
        auto pose_itr = this_node_->getPose();
        memcpy(pose, pose_itr, 16 * sizeof(double));
        Eigen::Matrix4d tran_matrix =
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(pose);
        target_barcode_.reset();
        target_barcode_ = nullptr;
        for (auto &barcode : v_barcode_) {
            int point_in_barcode_size = 0;
            for (int i = start_point_; i < end_point_; i++) {
                double x = address[i].x;
                double y = address[i].y;
                double z = address[i].z;
                if (std::abs(x) != INFINITY && std::abs(y) != INFINITY &&
                    std::abs(z) != INFINITY) {
                    Eigen::Vector4d point(x, y, z, 1);
                    point = tran_matrix * point;
                    if (barcode->checkBeScanned(point)) {
                        if (++point_in_barcode_size > success_size_) {
                            // TODO: 扫码成功
                            LOG_INFO("barcode size ,%d ; scaner = %s",
                                     point_in_barcode_size, name_.c_str());
                            LOG_INFO("find QRcode ,%s ; name = %s", barcode->getQRcode(), name_.c_str());
                            target_barcode_ = barcode;
                            notifyer_->onCode();
                            return;
                        }
                    }
                }
            }
        }
    }

   private:
    void getChildNode(Node *this_ptr) {
        if (this_ptr == nullptr) {
            return;
        }
        Field *children = this_ptr->getField("children");
        if (children != nullptr) {
            int cnt = children->getCount();
            for (int i = 0; i < cnt; i++) {
                Node *iter = children->getMFNode(i);

                getChildNode(iter);
            }
        }

        if (this_ptr->getTypeName().compare("Reflector") == 0) {
            v_barcode_.push_back(std::make_shared<WBarcode>(this_ptr));
            LOG_INFO("creat barcoder");
        }
    }

   private:
    static std::vector<std::shared_ptr<WBarcode>> v_barcode_;
    std::shared_ptr<WBarcode> target_barcode_;
    static bool enable_;

    Node *this_node_ = nullptr;
    std::string name_ = "";

    uint64_t cur_step_ = 0;
    std::shared_ptr<CoderNotifyer> notifyer_;
    int frequency_ = 0;
    int frequency_cnt_ = 0;
    Lidar *lidar_ = nullptr;
    int start_point_ = 0;
    int end_point_ = 0;
    int success_size_ = 0;
};

}  // namespace VNSim