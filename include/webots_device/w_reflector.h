/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 10:45:19
 * @FilePath:
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once

#include <webots/Node.hpp>
#include <shared_mutex>

#include "geometry/geometry.h"
#include "webots_device/w_base.h"
#include "logvn/logvn.h"

#include "geometry/geometry.h"
#include "lidar_simulation/high_reflector.h"
// #include "sim_data_flow/high_reflector.pb.h"

namespace VNSim {
using namespace webots;

class WReflector : public WBase {
   public:
    WReflector(std::string groud_name = "") : WBase() {
        node_ = super_->getFromDef(groud_name);
        if (node_ == nullptr) {
            LOG_INFO("there`s no reflector");
            return;
        }
        // 获取组内节点
        auto children = node_->getField("children");

        for (int i = 0; i < children->getCount(); i++) {
            Node *box = children->getMFNode(i);
            const double *translation =
                box->getField("translation")->getSFVec3f();
            const double *rotation = box->getField("rotation")->getSFRotation();
            const double *size = box->getField("size")->getSFVec3f();

            // 加2cm防止点在外侧
            double size_x = size[0] + 0.06;
            double size_y = size[1] + 0.06;
            double size_z = size[2] + 0.06;

            // 八个顶点 !顶点需要按照顺序
            std::vector<Eigen::Vector4d> p_list;
            p_list.push_back(
                Eigen::Vector4d(size_x / 2, size_y / 2, size_z / 2, 1));
            p_list.push_back(
                Eigen::Vector4d(-size_x / 2, size_y / 2, size_z / 2, 1));
            p_list.push_back(
                Eigen::Vector4d(-size_x / 2, -size_y / 2, size_z / 2, 1));
            p_list.push_back(
                Eigen::Vector4d(size_x / 2, -size_y / 2, size_z / 2, 1));

            p_list.push_back(
                Eigen::Vector4d(size_x / 2, -size_y / 2, -size_z / 2, 1));
            p_list.push_back(
                Eigen::Vector4d(-size_x / 2, -size_y / 2, -size_z / 2, 1));
            p_list.push_back(
                Eigen::Vector4d(-size_x / 2, size_y / 2, -size_z / 2, 1));
            p_list.push_back(
                Eigen::Vector4d(size_x / 2, size_y / 2, -size_z / 2, 1));

            // 变换顶点至 world_link
            Eigen::Matrix4d tran_matrix =
                createTransformMatrix(rotation, translation);
            for (int i = 0; i < p_list.size(); i++) {
                p_list[i] = tran_matrix * p_list[i];
                // LOG_INFO("i = %d,  %f,  %f,  %f", i, p_list[i].x(),
                //          p_list[i].y(), p_list[i].z());
            }
            // LOG_INFO("rotation %.2f, %.2f, %.2f, %.2f", rotation[0],
            //          rotation[1], rotation[2], rotation[3]);

            // LOG_INFO("matrix ");
            // LOG_INFO("%.2f, %.2f, %.2f, %.2f", tran_matrix(0, 0),
            //          tran_matrix(0, 1), tran_matrix(0, 2), tran_matrix(0,
            //          3));
            // LOG_INFO("%.2f, % .2f, % .2f, % .2f", tran_matrix(1, 0),
            //          tran_matrix(1, 1), tran_matrix(1, 2), tran_matrix(1,
            //          3));
            // LOG_INFO("%.2f, % .2f, % .2f, % .2f", tran_matrix(2, 0),
            //          tran_matrix(2, 1), tran_matrix(2, 2), tran_matrix(2,
            //          3));
            // LOG_INFO("%.2f, % .2f, % .2f, % .2f", tran_matrix(3, 0),
            //          tran_matrix(3, 1), tran_matrix(3, 2), tran_matrix(3,
            //          3));

            // 创建中心
            Eigen::Vector4d center(translation[0], translation[1],
                                   translation[2], 1);

            // 计算包围盒的轴
            std::vector<Eigen::Vector4d> axis_list;
            std::vector<double> min_list;
            std::vector<double> max_list;
            double minDotCube = 0;
            double maxDotCube = 0;

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

            // LOG_INFO("axis_list.size %d", axis_list.size());

            // 当前reflector获取完毕
            reflector_list_.push_back(
                Reflector(center, p_list, axis_list, min_list, max_list));
        }
    }

    // // 计算点和立方体在轴上的投影
    //                 double dotP = dotProduct(p, axis);
    //                   //
    //                   如果点的投影不在立方体的投影范围内，那么这个点就不在立方体内
    //                 if (dotP < minDotCube || dotP > maxDotCube) {
    //                     return false;
    //                 }
    // sim_data_flow::ReflectorMsg getReflectors() {
    //     sim_data_flow::ReflectorMsg result;
    //     for (auto reflector : reflector_list_) {
    //         sim_data_flow::Reflector *v_ptr = result.add_v_reflector();
    //         for (auto point_read : reflector.point_list) {
    //             sim_data_flow::Vector4 *point_set = v_ptr->add_reflector();
    //             point_set->set_x(point_read.x());
    //             point_set->set_y(point_read.y());
    //             point_set->set_z(point_read.z());
    //             point_set->set_w(point_read.w());
    //         }
    //     }
    //     return result;
    // }
    std::vector<Reflector> getReflectors() { return reflector_list_; }

    void spin() {}

   private:
    Node *node_ = nullptr;
    std::vector<Reflector> reflector_list_;
};

}  // namespace VNSim
