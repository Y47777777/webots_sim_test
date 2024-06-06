#pragma once

#include <webots/Node.hpp>
#include <webots/Lidar.hpp>

#include "time/time.h"
#include "NRLS.h"
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

            webots_point_could_address_ = lidar_->getPointCloud();
            size_of_layer_ = lidar_->getNumberOfLayers();
            size_of_point_cloud_ = lidar_->getNumberOfPoints();
            size_of_each_layer_ = size_of_point_cloud_ / size_of_layer_;

            point_cloud_.set_size_of_layer(size_of_layer_);
            point_cloud_.set_size_of_each_layer(size_of_each_layer_);
            point_cloud_.set_size_of_point_cloud(size_of_point_cloud_);
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

        if (is_sim_NRLS_ == false) {
            return;
        }

        const char *base_path = "../../plugins/lidar_scan_mode_config/";
        std::string file = std::string(base_path) + lidar_name_ + ".csv";
        struct LidarInfo input = {
            .horizontalResolution =
                node_->getField("horizontalResolution")->getSFInt32(),  // read
            .fieldOfView =
                node_->getField("fieldOfView")->getSFFloat(),  // read
            .verticalFieldOfView =
                node_->getField("verticalFieldOfView")->getSFFloat(),  // read
            .numberOfLayers =
                node_->getField("numberOfLayers")->getSFInt32()  // read
            // TODO: minZ
        };
        if (NRLS_ == nullptr) {
            NRLS_ = std::make_shared<NRLS>();
        }
        NRLS_->load(file.c_str(), input);
    }

    void moveLidar() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        if (translation_ptr_ == nullptr) {
            LOG_ERROR("can`t move %s", lidar_name_.c_str());
            return;
        }
        // TODO:mid 360 移动
    }

    bool checkDataReady() { return data_is_ready_; }

    void getLocalPointCloud(sim_data_flow::WBPointCloud &t_lidar,
                            int target_size = -1) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        t_lidar.CopyFrom(point_cloud_);
        data_is_ready_ = false;
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        // 根据频率拷贝数据
        int now_step_cnt = super_->getStepCnt() - start_step_;
        if (now_step_cnt % frequency_cnt_ != 0) {
            return;
        }
        data_is_ready_ = true;
        
        if (is_sim_NRLS_) {
            // 模拟非重复线扫
            NRLS_->doCopyProcess(lidar_, point_cloud_);
        } else {
            // 增加时间戳
            point_cloud_.set_timestamp(Timer::getInstance()->getTimeStamp());
            point_cloud_.clear_point_cloud();

            const LidarPoint *address = webots_point_could_address_;
            for (int i = 0; i < size_of_point_cloud_; i++) {
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
                }
            }
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

    const LidarPoint *webots_point_could_address_;
    sim_data_flow::WBPointCloud point_cloud_;

    double tf_translation_[3] = {0, 0, 0};
    double tf_rotation_[4] = {0, 0, 0, 0};
    std::shared_ptr<NRLS> NRLS_{nullptr};
    bool is_sim_NRLS_ = false;
    bool data_is_ready_ = false;
};

}  // namespace VNSim