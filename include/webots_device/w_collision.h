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

// TODO: 完善局部碰撞
class WCollision : public WBase {
   public:
    WCollision() : WBase() {
        root_ = super_->getRoot();

        // get all node
        getChildNode(root_);
        
    }
    ~WCollision() {}

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
        }

        Field *bounding_object = this_ptr->getField("boundingObject");
        if (bounding_object != nullptr) {
            bounding_object->removeSF();
        }
        Field *physics = this_ptr->getField("physics");
        if (physics != nullptr) {
            physics->removeSF();
            this_ptr->resetPhysics();
        }
    }

    void spin() {}

   private:
    Node *root_ = nullptr;
};

}  // namespace VNSim
