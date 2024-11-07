/**
 * @file w_collision.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-06-24
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <webots/Node.hpp>
#include <shared_mutex>
#include <nlohmann/json.hpp>

#include "webots_device/w_base.h"
#include "logvn/logvn.h"

namespace VNSim {
using namespace webots;

typedef struct WCollisionNode {
    WCollisionNode(Node *node_p = nullptr, Field *physics = nullptr) {
        node_ptr_ = node_p;
        physics_ptr_ = physics;
    }

    bool still_around_robot_ = false;
    std::string physics_str_;
    double tran_world[3] = {0, 0, 0};
    double tran_world_last[3] = {0, 0, 0};

    Node *node_ptr_ = nullptr;
    Field *physics_ptr_ = nullptr;
    void initWorldPosi(const double *t) {
        for (int i = 0; i < 3; ++i) {
            tran_world[i] = t[i];
            tran_world_last[i] = t[i];
        }
    }
};

class WCollision : public WBase {
   public:
    WCollision(bool is_shadow = true) : WBase() {
        is_shadow_ = is_shadow;
        root_ = super_->getRoot();
        robot_ = super_->getFromDef("RobotNode");

        // get all node
        getChildNode(root_);

        LOG_INFO("creat map size %d", m_node_.size());
    }
    ~WCollision() {}

   private:
    void getChildNode(Node *this_ptr) {
        if (this_ptr == nullptr) {
            return;
        }

        if (is_shadow_ == false) {
            auto node_type = this_ptr->getBaseTypeName();
            if (node_type.compare("Robot") == 0) {
                return;
            }
        }

        Field *children = this_ptr->getField("children");
        if (children != nullptr) {
            int cnt = children->getCount();
            for (int i = 0; i < cnt; i++) {
                Node *iter = children->getMFNode(i);
                getChildNode(iter);
            }
        }

        if (this_ptr->getTypeName().compare("HingeJoint") == 0) {
            LOG_INFO("there is HingeJoint");
            Field *iter = this_ptr->getField("endPoint");
            Node *endPoint = iter->getSFNode();
            getChildNode(endPoint);

        } else if (this_ptr->getTypeName().compare("SliderJoint") == 0) {
            LOG_INFO("there is SliderJoint");
            Field *iter = this_ptr->getField("endPoint");
            Node *endPoint = iter->getSFNode();
            getChildNode(endPoint);
        } else if (this_ptr->getTypeName().compare("Hinge2Joint") == 0) {
            LOG_INFO("there is Hinge2Joint");
            Field *iter = this_ptr->getField("endPoint");
            Node *endPoint = iter->getSFNode();
            getChildNode(endPoint);
        }

        if (is_shadow_) {
            // shadow 删除所有碰撞属性,物理属性
            Field *bounding_object = this_ptr->getField("boundingObject");
            if (bounding_object != nullptr) {
                bounding_object->removeSF();
            }
            Field *physics = this_ptr->getField("physics");
            if (physics != nullptr) {
                physics->removeSF();
                this_ptr->resetPhysics();
            }
        } else {
            Field *physics = this_ptr->getField("physics");
            if (physics != nullptr) {
                Node *physics_node = physics->getSFNode();

                if (physics_node != nullptr) {
                    WCollisionNode node(this_ptr, physics);
                    node.initWorldPosi(this_ptr->getPosition());

                    node.physics_str_ = physics_node->exportString();

                    // 记录当前节点物理属性
                    m_node_.insert(std::make_pair(idx_, node));
                    idx_++;

                    // bounding_object->removeSF();
                    physics->removeSF();
                    this_ptr->resetPhysics();
                }
            }
        }
    }

    void spin() {
        const double *robot_tran = robot_->getPosition();
        const float BASE_RANGE = 3;
        const float POSI_THRESHOLD = 0.001;

        for (auto it = m_node_.begin(); it != m_node_.end(); ++it) {
            it->second.still_around_robot_ = false;
            const double *node_tran = it->second.tran_world;
            // 判断是否发生位移
            const double *cur_pose = it->second.node_ptr_->getPosition();
            if (std::abs(cur_pose[0] - node_tran[0]) >= POSI_THRESHOLD ||
                std::abs(cur_pose[1] - node_tran[1]) >= POSI_THRESHOLD ||
                std::abs(cur_pose[2] - node_tran[2]) >= POSI_THRESHOLD) {
                // 世界坐标是否发生变化
                memcpy(it->second.tran_world, cur_pose, sizeof(*cur_pose) * 3);
                // std::cout << "位置发生改变:" << it->second.node_ptr_->getField("name")->getSFString() << std::endl;
                // std::cout << "节点:"<< it->second.node_ptr_->getField("name")->getSFString() << "位置差:" <<
                // std::abs(robot_tran[0] - node_tran[0]) << " " << std::abs(robot_tran[1] - node_tran[1]) << " " <<
                // std::abs(robot_tran[2] - node_tran[2]) << std::endl; std::cout << "solid的位置："<< node_tran[0] << "
                // ," << node_tran[1] << " ," << node_tran[2] << std::endl; std::cout << "robot的位置："<< robot_tran[0]
                // << " ," << robot_tran[1] << " ," << robot_tran[2] << std::endl;
            }
            // 节点在机器人范围内
            if (std::abs(robot_tran[0] - node_tran[0]) <= BASE_RANGE &&
                std::abs(robot_tran[1] - node_tran[1]) <= BASE_RANGE) {
                // 不在open list 中的添加节点
                if (m_open_list_.find(it->first) == m_open_list_.end()) {
                    if (it->second.physics_ptr_->getSFNode() == nullptr) {
                        it->second.physics_ptr_->importSFNodeFromString(it->second.physics_str_);

                        LOG_INFO("add idx %d", it->first);
                    }
                    // if (it->second.bounding_ptr_->getSFNode() == nullptr) {
                    //     it->second.bounding_ptr_->importSFNodeFromString(
                    //         it->second.bounding_str_);
                    // }
                    m_open_list_[it->first] = &it->second;
                }
                it->second.still_around_robot_ = true;
            }
        }

        for (auto it = m_open_list_.begin(); it != m_open_list_.end();) {
            WCollisionNode *node_ptr = it->second;
            if (node_ptr->still_around_robot_ == false) {
                // node_ptr->bounding_ptr_->removeSF();
                node_ptr->physics_ptr_->removeSF();
                node_ptr->node_ptr_->resetPhysics();
                it = m_open_list_.erase(it);  // erase returns the next iterator

                LOG_INFO("delete idx %d", it->first);
            } else {
                ++it;
            }
        }
    }

   private:
    Node *root_ = nullptr;
    Node *robot_ = nullptr;
    uint64_t idx_ = 0;
    bool is_shadow_ = true;

    std::map<uint64_t, WCollisionNode> m_node_;
    std::map<uint64_t, WCollisionNode *> m_open_list_;
};

}  // namespace VNSim
