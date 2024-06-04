#pragma once

#include <webots/Node.hpp>
#include <webots/Lidar.hpp>

#include "webots_device/w_base.h"
#include "sim_data_flow/point_cloud.pb.h"
#include "logvn/logvn.h"

namespace VNSim {
using namespace webots;
class WLidar : public WBase {
   public:
    /*
     * @brief  构建 Lidar
     *
     * @param[lidar_name]  webots 下 lidar名字
     * @param[pose_name]   lidar_name 外层pose
     * @param[frequency]   ladir 频率 (ms)
     */
    WLidar(std::string lidar_name, std::string pose_name = "",
           int frequency = 100)
        : WBase() {
        // creat lidar_name
        lidar_name_ = lidar_name;
        frequency_ = frequency;

        lidar_ = super_->getLidar(lidar_name);
        if (lidar_ == nullptr) {
            LOG_ERROR("%s is nullptr", lidar_name.c_str());
            return;
        }

        lidar_->enable(frequency_);
        lidar_->enablePointCloud();

        webots_point_could_address_ = lidar_->getPointCloud();
        int size_of_layer = lidar_->getNumberOfLayers();
        int size_of_point_cloud = lidar_->getNumberOfPoints();
        int size_of_each_layer = size_of_point_cloud / size_of_layer;

        point_cloud_.set_size_of_layer(size_of_layer);
        point_cloud_.set_size_of_each_layer(size_of_each_layer);
        point_cloud_.set_size_of_point_cloud(size_of_point_cloud);

        // creat pose
        Node *node_ = super_->getFromDef(pose_name);
        if (node_ != nullptr) {
            translation_ptr_ = node_->getField("translation");
            rotation_ptr_ = node_->getField("rotation");

            memcpy(tf_rotation_, rotation_ptr_->getSFRotation(),
                   4 * sizeof(tf_rotation_[0]));
            memcpy(tf_translation_, translation_ptr_->getSFVec3f(),
                   3 * sizeof(tf_translation_[0]));

            LOG_INFO("pose node :", pose_name.c_str());
            LOG_INFO("pose rotation %.3f, %.3f, %.3f, %.3f", tf_rotation_[0],
                     tf_rotation_[1], tf_rotation_[2], tf_rotation_[3]);

            LOG_INFO("pose translation %.3f, %.3f, %.3f", tf_translation_[0],
                     tf_translation_[1], tf_translation_[2]);
        }
    }

    ~WLidar() {}

    /*
     * @brief  使能非重复线扫模拟 Non-repetitive Line Scan LIDAR
     *
     * @param[flag]  true->使能    ，false->失能
     */
    void setSimulationNRLS(bool flag) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        is_sim_NRLS_ = flag;

        point_cloud_.clear_point_cloud();
        // TODO:完善非重复线扫的模拟
        // set size();
    }

    void moveLidar() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        if (translation_ptr_ == nullptr) {
            LOG_ERROR("");
            return;
        }
        // TODO:mid 360 移动
    }

    // TODO: delete
    const Lidar *getLidarPtr() { return lidar_; }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        // TODO: 模拟非重复线扫
        if (is_sim_NRLS_) {
            // 拷贝结束后直接返回

        } else {
            // // copy to stash
            point_cloud_.clear_point_cloud();
            int size = point_cloud_.size_of_point_cloud();
            const LidarPoint *address = webots_point_could_address_;
            for (int i = 0; i < size; i++) {
                sim_data_flow::LidarPoint *point =
                    point_cloud_.add_point_cloud();
                point->set_x(address[i].x);
                point->set_y(address[i].y);
                point->set_z(address[i].z);
                point->set_time(address[i].time);
                point->set_layer_id(address[i].layer_id);
            }
        }
    }

   private:

   private:
    std::string lidar_name_ = "";
    Lidar *lidar_ = nullptr;
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;

    int frequency_ = 0;

    const LidarPoint *webots_point_could_address_;
    sim_data_flow::WBPointCloud point_cloud_;

    double tf_translation_[3] = {0, 0, 0};
    double tf_rotation_[4] = {0, 0, 0, 0};

    bool is_sim_NRLS_ = false;
};

}  // namespace VNSim