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

#include "geometry/geometry.h"
#include "time/time.h"
#include "NRLS.h"
#include "webots_device/w_base.h"
#include "sim_data_flow/point_cloud.pb.h"
#include "logvn/logvn.h"

namespace VNSim {
using namespace webots;

typedef struct VertivalFov {
    // TODO: 非重复线扫数据
    double begin = 0;
    double end = 0;
};
class WLidar : public WBase {
   public:
    /**
     * @brief Construct a new WLidar object
     *
     * @param[in] lidar_name  webots 下 lidar名字
     * @param[in] pose_name   lidar_name 外层pose
     * @param[in] frequency   ladir 频率 (ms)
     */
    WLidar(std::string lidar_name, std::string pose_name = "",
           int frequency = 100)
        : WBase() {
        // creat lidar
        {
            lidar_name_ = lidar_name;
            frequency_ = frequency;
            frequency_cnt_ = std::round(double(frequency_ / step_duration_));

            lidar_ = super_->getLidar(lidar_name);
            if (lidar_ == nullptr) {
                LOG_ERROR("%s is nullptr", lidar_name.c_str());
                return;
            }

            lidar_->enable(frequency_);
            lidar_->enablePointCloud();

            webots_points_address_ = lidar_->getPointCloud();
            size_of_layer_ = lidar_->getNumberOfLayers();
            size_of_point_cloud_ = lidar_->getNumberOfPoints();
            size_of_each_layer_ = size_of_point_cloud_ / size_of_layer_;

            point_cloud_.set_size_of_layer(size_of_layer_);
            point_cloud_.set_size_of_each_layer(size_of_each_layer_);
            point_cloud_.set_size_of_point_cloud(size_of_point_cloud_);

            // 获取每层的地址
            v_webots_address_.clear();
            for (int i = 0; i < size_of_layer_; i++) {
                v_webots_address_.push_back(lidar_->getLayerPointCloud(i));
            }
        }

        // creat pose
        {
            node_ = super_->getFromDef(pose_name);
            if (node_ != nullptr) {
                translation_ptr_ = node_->getField("translation");
                rotation_ptr_ = node_->getField("rotation");

                memcpy(tf_rotation_, rotation_ptr_->getSFRotation(),
                       4 * sizeof(tf_rotation_[0]));
                memcpy(tf_translation_, translation_ptr_->getSFVec3f(),
                       3 * sizeof(tf_translation_[0]));

                LOG_INFO("pose node :", pose_name.c_str());
                LOG_INFO("pose rotation %.3f, %.3f, %.3f, %.3f",
                         tf_rotation_[0], tf_rotation_[1], tf_rotation_[2],
                         tf_rotation_[3]);

                LOG_INFO("pose translation %.3f, %.3f, %.3f",
                         tf_translation_[0], tf_translation_[1],
                         tf_translation_[2]);
            }
        }

        // 设置雷达数据生成其实步数，间隔雷达数据
        start_step_ = super_->getStepCnt();
        super_->step(step_duration_);
        data_is_ready_ = false;
    }

    /**
     * @brief Set the Simulation N R L S object
     *
     * @param[in] path  查找表 （base 路径已写死）
     */
    void setSimulationNRLS(std::string path) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);

        is_sim_NRLS_ = true;
        const char *base_path = "../../plugins/lidar_scan_mode_config/";
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

        if (NRLS_->load(file.c_str(), input) != 0) {
            is_sim_NRLS_ = false;
            return;
        }
    }

    void setFov(const VertivalFov ver_fov) {
        start_layer_ = 0;
        end_layer_ = size_of_layer_;
        if (fabs(ver_fov.begin - ver_fov.end) > 0.01) {
            fov_vertical_enable_ = true;

            // 计算起始层
            double vertical_fov = lidar_->getVerticalFov();
            double resolution = size_of_layer_ / vertical_fov;
            double start_angle = PI / 2 - ver_fov.end;
            start_layer_ = std::round(start_angle * resolution);

            // 计算终点
            double end_angle = PI / 2 - ver_fov.begin;
            end_layer_ = std::round(end_angle * resolution);

            if (start_layer_ < 0) {
                LOG_ERROR("end_layer error :%d", start_layer_);
                start_layer_ = 0;
            }
            if (end_layer_ > size_of_layer_) {
                LOG_INFO("start_layer_:%d", end_layer_);
                end_layer_ = size_of_layer_;
            }
            LOG_INFO("start_layer = %d, end_layer = %d", start_layer_,
                     end_layer_);
        }
    }

    /*
     * @brief  移动雷达(叉端mid360 随动)
     *
     * @param[values]  Vec3f
     */
    void moveLidar(const double values[3]) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        // check lidar can be move
        if (translation_ptr_ == nullptr) {
            LOG_ERROR("can`t move %s", lidar_name_.c_str());
            return;
        }
        translation_ptr_->setSFVec3f(values);
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
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        result.CopyFrom(point_cloud_);
        data_is_ready_ = false;
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        // 根据频率拷贝数据
        int cur_step_cnt = super_->getStepCnt() - start_step_;
        if (cur_step_cnt % frequency_cnt_ != 0) {
            return;
        }
        data_is_ready_ = true;

        if (is_sim_NRLS_) {
            // 模拟非重复线扫
            NRLS_->simulation(webots_points_address_, point_cloud_);
        } else {
            // 增加时间戳
            point_cloud_.set_timestamp(Timer::getInstance()->getTimeStamp());
            point_cloud_.clear_point_cloud();

            const LidarPoint *address = webots_points_address_;
            double x = 0;
            double y = 0;
            double z = 0;

            // 按层转存 只取fov内
            for (int i = start_layer_; i < end_layer_; i++) {
                if (i > v_webots_address_.size()) {
                    LOG_ERROR("%s, i > v_webots_address_.size()",
                              lidar_name_.c_str());
                    return;
                }

                const LidarPoint *iter = v_webots_address_[i];
                for (int j = 0; j < size_of_each_layer_; j++) {
                    x = iter[j].x;
                    y = iter[j].y;
                    z = iter[j].z;
                    if (std::abs(x) != INFINITY && std::abs(y) != INFINITY &&
                        std::abs(z) != INFINITY) {
                        sim_data_flow::LidarPoint *point =
                            point_cloud_.add_point_cloud();
                        point->set_x(x);
                        point->set_y(y);
                        point->set_z(z);
                        point->set_time(i);
                        point->set_layer_id(iter[i].layer_id);
                    }
                }
            }
            // for (int i = 0; i < size_of_point_cloud_; i++) {
            //     double x = address[i].x;
            //     double y = address[i].y;
            //     double z = address[i].z;
            //     if (std::abs(x) != INFINITY && std::abs(y) != INFINITY &&
            //         std::abs(z) != INFINITY) {
            //         sim_data_flow::LidarPoint *point =
            //             point_cloud_.add_point_cloud();
            //         point->set_x(x);
            //         point->set_y(y);
            //         point->set_z(z);
            //         point->set_time(address[i].time);
            //         point->set_layer_id(address[i].layer_id);
            //     }
            // }
        }
    }

   private:
    std::string lidar_name_ = "";
    Lidar *lidar_ = nullptr;
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;
    Node *node_ = nullptr;

    int frequency_ = 0;
    int frequency_cnt_ = 0;

    int size_of_layer_ = 0;
    int size_of_point_cloud_ = 0;
    int size_of_each_layer_ = 0;

    const LidarPoint *webots_points_address_;
    std::vector<const LidarPoint *> v_webots_address_;
    sim_data_flow::WBPointCloud point_cloud_;

    double tf_translation_[3] = {0, 0, 0};
    double tf_rotation_[4] = {0, 0, 0, 0};
    std::shared_ptr<NRLS> NRLS_{nullptr};
    bool is_sim_NRLS_ = false;
    bool data_is_ready_ = false;

    int last_point_ = {-1};

    // z轴fov
    bool fov_vertical_enable_ = false;
    int start_layer_ = 0;  // 开始拷贝数据的 层
    int end_layer_ = 0;    //
};

}  // namespace VNSim