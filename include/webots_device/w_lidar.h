#pragma once

#include <webots/Node.hpp>
#include <webots/Lidar.hpp>

#include "webots_device/w_base.h"
#include "logvn/logvn.h"

namespace VNSim {
using namespace webots;
class WLidar : public WBase {
   public:
    /*
     * @brief  构建 Lidar
     *
     * @param[motor_name]  webots 下 lidar名字
     * @param[pose_name]   lidar 外层pose
     */
    WLidar(std::string lidar, std::string pose_name = "") : WBase() {
        // creat lidar
        lidar_ = super_->getLidar(lidar);
        if (lidar_ == nullptr) {
            LOG_ERROR("lidar_ptr is nullptr");
            return;
        }

        lidar_->enable(5);
        lidar_->enablePointCloud();

        layer_count_ = lidar_->getNumberOfLayers();
        point_cloud_size_ = lidar_->getNumberOfPoints();

        for (int i = 0; i < layer_count_; i++) {
            v_point_ptr_.push_back(lidar_->getLayerPointCloud(i));
        }

        LOG_INFO("%s creat success", lidar.c_str());

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

    const Lidar *getLidarPtr() { return lidar_; }

    void getPointCloudPtr(std::vector<const LidarPoint *> &v_ptr) {
        v_ptr.clear();
        for (int i = 0; i < v_point_ptr_.size(); ++i) {
            v_ptr.push_back(v_point_ptr_[i]);
        }
    }

    void getPointCloud2() {
        // TODO: exchange to PointCloud2
    }

    void getPointCloudAllMessage() {
        // TODO:exchange to PointCloudAllMessage
    }

    void moveLidar() {
        // TODO: move lidat
    }

    void spin() {}

   private:
    Lidar *lidar_ = nullptr;
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;

    std::vector<const LidarPoint *> v_point_ptr_;
    int layer_count_ = 0;
    int point_cloud_size_ = 0;

    double tf_translation_[3] = {0, 0, 0};
    double tf_rotation_[4] = {0, 0, 0, 0};
};

}  // namespace VNSim