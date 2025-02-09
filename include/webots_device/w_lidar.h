/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:08
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 11:26:16
 * @FilePath: /webots_ctrl/include/webots_device/w_lidar.h
 * @Description:
 *               webots lidar接口
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */

#pragma once

#include <webots/Node.hpp>
#include <webots/Lidar.hpp>

#include "logvn/logvn.h"
#include "geometry/geometry.h"

#include "sim_data_flow/point_cloud.pb.h"

#include "webots_device/w_base.h"
#include "webots_device/w_pose.h"

#include "NRLS.h"

namespace VNSim {
using namespace webots;

typedef struct VertivalFov {
    // 以z轴为0点，\in [0,PI]
    double begin = 0;  // 起始角度
    double end = 0;    // 终止角度
};

class WLidar : public WBase {
   public:
    /**
     * @brief Construct a new WLidar object
     *
     * @param[in] lidar_name  webots 下 lidar名字
     * @param[in] pose_name   lidar_name 外层pose
     * @param[in] frequency   ladir 频率 (ms)
     * @param[in] connecting_rod   连杆长度
     */
    WLidar(std::string lidar_name, std::shared_ptr<WPose> pose_ptr, int frequency = 100, bool enable = true,
           double connecting_rod = 0.241)
        : WBase() {
        // creat lidar
        {
            lidar_name_ = lidar_name;
            frequency_ = frequency;
            frequency_cnt_ = std::round(double(frequency_ / step_duration_));
            pose_ptr_ = pose_ptr;

            lidar_ = super_->getLidar(lidar_name);
            if (lidar_ == nullptr) {
                LOG_ERROR("%s is nullptr", lidar_name.c_str());
                return;
            }

            if (enable) {
                lidar_->enable(frequency_);
                lidar_->enablePointCloud();

                webots_points_address_ = lidar_->getPointCloud();
                size_of_layer_ = lidar_->getNumberOfLayers();
                size_of_point_cloud_ = lidar_->getNumberOfPoints();
                size_of_each_layer_ = size_of_point_cloud_ / size_of_layer_;

                point_cloud_.set_size_of_layer(size_of_layer_);
                point_cloud_.set_size_of_each_layer(size_of_each_layer_);
                point_cloud_.set_size_of_point_cloud(size_of_point_cloud_);
                point_cloud_.set_name(lidar_name);

                // 获取每层的地址
                // v_webots_address_.clear();
                // for (int i = 0; i < size_of_layer_; i++) {
                //     v_webots_address_.push_back(lidar_->getLayerPointCloud(i));
                // }
                start_point_ = 0;
                end_point_ = size_of_point_cloud_;
            }
        }

        // creat pose
        node_ = super_->getFromDevice(lidar_);
        if (node_ != nullptr) {
            translation_ptr_ = node_->getField("translation");
            rotation_ptr_ = node_->getField("rotation");

            memcpy(tf_rotation_, rotation_ptr_->getSFRotation(),
                   4 * sizeof(tf_rotation_[0]));
            memcpy(tf_translation_, translation_ptr_->getSFVec3f(),
                   3 * sizeof(tf_translation_[0]));
            extrinsic_ = getExtrinsicFromDef();

            LOG_INFO("pose rotation %.3f, %.3f, %.3f, %.3f", tf_rotation_[0],
                     tf_rotation_[1], tf_rotation_[2], tf_rotation_[3]);

            LOG_INFO("pose translation %.3f, %.3f, %.3f", tf_translation_[0],
                     tf_translation_[1], tf_translation_[2]);

            connecting_rod_ = connecting_rod;
        }

        // 设置雷达数据生成其实步数，间隔雷达数据
        // FIXME: +1 为错步发送
        start_step_ = super_->getStepCnt();
        super_->step(step_duration_);
        data_is_ready_ = false;
    }

    /**
     * @brief Set the Simulation N R L S object
     *
     * @param[in] path  查找表 （base 路径已写死）
     */
    void setSimulationNRLS(std::string path,
                           size_t nrls_cloud_size = MID360_ONCE_CLOUD_SIZE) {
        AutoAtomicLock lock(spin_mutex_);

        is_sim_NRLS_ = true;
        //const char *base_path = "../../plugins/lidar_scan_mode_config/";
        const char *base_path = "../../configs/lidar_scan_mode_config/";
        std::string file = std::string(base_path) + path;
        LOG_INFO(file.c_str());
        point_cloud_.clear_point_cloud();
        LidarInfo input = {
            .horizontalResolution = lidar_->getHorizontalResolution(),
            .fieldOfView = lidar_->getFov(),
            .verticalFieldOfView = lidar_->getVerticalFov(),
            .numberOfLayers = lidar_->getNumberOfLayers(),
        };

        if (NRLS_ == nullptr) {
            NRLS_ = std::make_shared<NRLS>();
        }

        if (NRLS_->load(file.c_str(), input, nrls_cloud_size) != 0) {
            is_sim_NRLS_ = false;
            return;
        }
    }

    /**
     * @brief Set the Fov object
     *
     * @param[in] ver_fov fov配置 以z轴为0点，\in [0,PI]
     */
    void setFov(const VertivalFov ver_fov) {
        int start_layer = 0;
        int end_layer = size_of_layer_;
        if (fabs(ver_fov.begin - ver_fov.end) > 0.01) {
            fov_vertical_enable_ = true;

            double vertical_fov = lidar_->getVerticalFov();
            double hard_start_angle = (PI - vertical_fov) / 2;  // 硬件起始
            double resolution = size_of_layer_ / vertical_fov;  // (线/角度)

            // 计算起始线，终止线
            start_layer =
                std::round((ver_fov.begin - hard_start_angle) * resolution);
            end_layer =
                std::round((ver_fov.end - hard_start_angle) * resolution);

            if (start_layer < 0) {
                LOG_ERROR("%s hardware vetical fov error start_layer: %d",
                          lidar_name_.c_str(), start_layer);
                start_layer = 0;
            }

            if (end_layer > size_of_layer_) {
                LOG_ERROR("%s hardware vetical fov error size_of_layer: %d",
                          lidar_name_.c_str(), end_layer);
                end_layer = size_of_layer_;
            }

            // result
            start_point_ = start_layer * size_of_each_layer_;
            end_point_ = end_layer * size_of_each_layer_;
        }
    }

    /**
     * @brief Get the Frequency object
     *
     * @return int 雷达频率
     */
    int getSleepTime() {
        int sleep = frequency_ - 10;
        sleep = sleep < 0 ? 0 : sleep;
        return sleep;
    }

    /*
     * @brief  移动雷达(叉端mid360 随动)
     *
     * @param[values]  Vec3f
     */
    void moveLidar(const double &values) {
        AutoAtomicLock lock(spin_mutex_);
        // check lidar can be move

        if (translation_ptr_ == nullptr) {
            LOG_ERROR("can`t move %s", lidar_name_.c_str());
            return;
        }
        double translation[3] = {tf_translation_[0], tf_translation_[1],
                                 tf_translation_[2]};

        if (values < connecting_rod_) {
            translation[2] = tf_translation_[2] - values;
        } else {
            translation[2] = tf_translation_[2] - (connecting_rod_);
        }

        translation_ptr_->setSFVec3f(translation);
    }

    Eigen::Matrix4d getMatrixFromLidar() {
        AutoAtomicLock lock(spin_mutex_);
        return extrinsic_;
    }

    Eigen::Matrix4d getExtrinsicFromDef(const std::string define = "RobotNode") {
        AutoAtomicLock lock(spin_mutex_);

        auto robot_node = super_->getFromDef(define);
        if (robot_node == nullptr) {
            return Eigen::Matrix4d::Identity();
        }

        Eigen::Matrix4d result = Eigen::Matrix4d(node_->getPose(robot_node)).transpose();
        return result;
    }

    /**
     * @brief 检查雷达数据更新状态
     *
     * @return true
     * @return false
     */
    bool checkDataReady() { return data_is_ready_; }

    /**
     * @brief Get the Local Point Cloud object
     *
     * @param[out] result
     * @param[in]  target_size  FIXME: 如果需要拷贝指定大小
     */
    void getLocalPointCloud(sim_data_flow::WBPointCloud &result,
                            int target_size = -1) {
        AutoAtomicLock lock(spin_mutex_);
        result.CopyFrom(point_cloud_);
        result.set_timestamp(time_stamp_);
        data_is_ready_ = false;
    }

    void spin() {
        // 根据频率拷贝数据
        int cur_step_cnt = super_->getStepCnt() - start_step_;
        if (cur_step_cnt % frequency_cnt_ != 0) {
            return;
        }

        // 开始拷贝数据
        data_is_ready_ = true;
        webots_points_address_ = lidar_->getPointCloud();

        point_cloud_.clear_point_cloud();
        point_cloud_.set_name(lidar_name_);

        if (is_sim_NRLS_) {
            // 模拟非重复线扫
            NRLS_->simulation(webots_points_address_, point_cloud_,
                              lidar_name_);
        } else {
            const LidarPoint *address = webots_points_address_;
            double x = 0;
            double y = 0;
            double z = 0;

            // 转存 只取fov内
            for (int i = start_point_; i < end_point_; i++) {
                double x = address[i].x;
                double y = address[i].y;
                double z = address[i].z;
                if (std::abs(x) != INFINITY && std::abs(y) != INFINITY &&
                    std::abs(z) != INFINITY) {
                    sim_data_flow::LidarPoint *point =
                        point_cloud_.add_point_cloud();
                    point->set_x(x);
                    point->set_y(y);
                    point->set_z(z);
                    point->set_time(address[i].time);
                    point->set_layer_id(address[i].layer_id);

                    point->set_intensity(100);
                }
            }
        }
        if (pose_ptr_ != nullptr)
            time_stamp_ = pose_ptr_->getTimeStamp();
    }

   private:
    std::string lidar_name_ = "";
    Lidar *lidar_ = nullptr;
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;
    Node *node_ = nullptr;

    int frequency_ = 0;
    int frequency_cnt_ = 0;

    uint64_t time_stamp_ = 0;
    std::shared_ptr<WPose> pose_ptr_;

    int size_of_layer_ = 0;
    int size_of_point_cloud_ = 0;
    int size_of_each_layer_ = 0;

    const LidarPoint *webots_points_address_;
    std::vector<const LidarPoint *> v_webots_address_;
    sim_data_flow::WBPointCloud point_cloud_;

    double tf_translation_[3] = {0, 0, 0};
    double tf_rotation_[4] = {0, 0, 0, 0};
    Eigen::Matrix4d extrinsic_;
    std::shared_ptr<NRLS> NRLS_{nullptr};
    bool is_sim_NRLS_ = false;
    bool data_is_ready_ = false;

    int last_point_ = {-1};

    // z轴fov
    bool fov_vertical_enable_ = false;

    int start_point_ = 0;  // 开始拷贝数据的 层
    int end_point_ = 0;    //

    // 雷达移动连杆长度
    double connecting_rod_ = 0.241;
};

// for (int i = start_layer; i < end_layer; i++) {
//     if (i > v_webots_address_.size()) {
//         LOG_ERROR("%s, i > v_webots_address_.size()",
//                   lidar_name_.c_str());
//         return;
//     }

//     const LidarPoint *iter = v_webots_address_[i];
//     for (int j = 0; j < size_of_each_layer_; j++) {
//         x = iter[j].x;
//         y = iter[j].y;
//         z = iter[j].z;
//         if (std::abs(x) != INFINITY && std::abs(y) != INFINITY &&
//             std::abs(z) != INFINITY) {
//             sim_data_flow::LidarPoint *point =
//                 point_cloud_.add_point_cloud();
//             point->set_x(x);
//             point->set_y(y);
//             point->set_z(z);
//             point->set_time(i);
//             // point->set_layer_id(iter[i].layer_id);
//             //
//             if(ReflectorChecker::getInstance()->checkInReflector(lidar_name_,point)){
//             //     point->set_intensity(200);
//             // }else{
//             //     point->set_intensity(100);
//             // }
//         }
//     }
// }
}  // namespace VNSim