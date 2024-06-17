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
            double size_x = size[0] + 0.04;
            double size_y = size[1] + 0.04;
            double size_z = size[2] + 0.04;

            // 八个顶点 !顶点需要按照顺序
            std::vector<Eigen::Vector4d> p_list;
            p_list.push_back(Eigen::Vector4d( size_x / 2,  size_y / 2, size_z / 2, 1));
            p_list.push_back(Eigen::Vector4d(-size_x / 2,  size_y / 2, size_z / 2, 1));
            p_list.push_back(Eigen::Vector4d(-size_x / 2, -size_y / 2, size_z / 2, 1));
            p_list.push_back(Eigen::Vector4d( size_x / 2, -size_y / 2, size_z / 2, 1));
            

            p_list.push_back(Eigen::Vector4d( size_x / 2,  size_y / 2, -size_z / 2, 1));
            p_list.push_back(Eigen::Vector4d(-size_x / 2,  size_y / 2, -size_z / 2, 1));
            p_list.push_back(Eigen::Vector4d(-size_x / 2, -size_y / 2, -size_z / 2, 1));
            p_list.push_back(Eigen::Vector4d( size_x / 2, -size_y / 2, -size_z / 2, 1));
            

            // 变换顶点至 world_link
            Eigen::Matrix4d tran_matrix =
                createTransformMatrix(rotation, translation);
            for (int i = 0; i < p_list.size(); i++) {
                p_list[i] = tran_matrix * p_list[i];
                LOG_INFO("i = %d,  %f,  %f,  %f", i, p_list[i].x(),
                         p_list[i].y(), p_list[i].z());
            }

            // 当前reflector获取完毕
            Eigen::Vector4d center(translation[0], translation[1],
                                   translation[2], 1);
            reflector_list_.push_back(Reflector(center, p_list));
        }
    }

    std::vector<Reflector> getReflectors() { return reflector_list_; }

    void spin() {}

   private:
    Node *node_ = nullptr;
    std::vector<Reflector> reflector_list_;
};

}  // namespace VNSim
