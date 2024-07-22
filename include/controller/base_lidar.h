/**
 * @file base_lidar.h
 * @author weijchen (weijchen@visionnav.com)
 * @brief  shadow 基类
 * @version 0.1
 * @date 2024-07-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma

#include "base_ctrl.h"

#include "sim_data_flow/point_cloud.pb.h"
#include "sim_data_flow/pose.pb.h"

#include "webots_device/w_lidar.h"
#include "webots_device/w_pose.h"

namespace VNSim {
class BaseLidarControl : public BaseController {
   protected:
    explicit BaseLidarControl(std::string ecal_name, QObject *parent = nullptr)
        : BaseController(ecal_name, parent) {}

    bool sendPointCloud(std::string topic, std::shared_ptr<WLidar> lidar_ptr,
                        std::shared_ptr<WPose> pose_ptr,
                        std::string multi_topic = "") {
        if (lidar_ptr == nullptr) {
            return false;
        }

        if (!lidar_ptr->checkDataReady()) {
            Timer::getInstance()->sleep<microseconds>(5);
            return false;
        }

        Timer lidar_alarm;
        lidar_alarm.alarmTimerInit(lidar_ptr->getSleepTime());

        sim_data_flow::WBPointCloud payload;

        lidar_ptr->getLocalPointCloud(payload);

        payload.set_timestamp(pose_ptr->getTimeStamp());

        // 在数量大的情况下约为10ms
        for (int i = 0; i < payload.point_cloud_size(); i++) {
            if (ReflectorChecker::getInstance()->checkInReflector(
                    payload.name(), &payload.point_cloud().at(i))) {
                payload.mutable_point_cloud()->at(i).set_intensity(200);
            }
        }
        uint8_t buf[payload.ByteSize()];
        payload.SerializePartialToArray(buf, payload.ByteSize());
        ecal_ptr_->send(topic.c_str(), buf, payload.ByteSize());

        if (!multi_topic.empty()) {
            std::cout << multi_topic;
            sim_data_flow::WBPointCloud payload_multi;
            // pbToBaseLink(payload, payload_multi,
            //              lidar_ptr->getMatrixFromLidar());

            uint8_t buf_mutil[payload_multi.ByteSize()];
            payload_multi.SerializePartialToArray(buf_mutil,
                                                  payload_multi.ByteSize());
            ecal_ptr_->send(multi_topic.c_str(), buf_mutil,
                            payload_multi.ByteSize());
        }

        lidar_alarm.wait();
        return true;
    }

    void pbToBaseLink(const sim_data_flow::WBPointCloud &payload,
                      sim_data_flow::WBPointCloud &result,
                      Eigen::Matrix4d &matrix) {
        // 复制点云
        result.CopyFrom(payload);

        // 遍历点云
        size_t point_size = result.point_cloud().size();
        for (int i = 0; i < point_size; i++) {
            auto p = result.mutable_point_cloud()->mutable_data()[i];
            Eigen::Vector4d point(p->x(), p->y(), p->z(), 1);

            // 转换至Base_link 下
            point = matrix * point;
            p->set_x(point.x());
            p->set_y(point.y());
            p->set_z(point.z());
        }
    }
};
}  // namespace VNSim
