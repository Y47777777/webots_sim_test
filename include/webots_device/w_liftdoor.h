/**
 * @file w_liftdoor.h
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
#include "sim_data_flow/transfer.pb.h"

namespace VNSim {
using namespace webots;

struct WNodeInfo {
    WNodeInfo(Node *node_ptr = nullptr, Field *customdata_p = nullptr, Field *manumode_p = nullptr)
        : node_ptr_(node_ptr), customdata_f_ptr_(customdata_p), manumode_f_ptr_(manumode_p) {}

    Node *node_ptr_ = nullptr;
    double position[3] = {0, 0, 0};
    bool open_ = false;
    Field *customdata_f_ptr_ = nullptr;
    Field *manumode_f_ptr_ = nullptr;
};

class WLiftDoor : public WBase {
   public:
    WLiftDoor(bool direct) : WBase(), direction(direct) {
        root_ = super_->getRoot();

        robot_ = super_->getFromDef("RobotNode");

        idx = 0;

        // get all node
        getLiftDoorNode();

        // LOG_INFO("size of tran map %d", m_tanfer_.size());
    }

    void getTag(sim_data_flow::MTransfer &tanfer_msgs) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        tanfer_msgs.clear_map();

        for (const auto &[id, node] : m_liftdoor_updated_) {
            auto ptr = tanfer_msgs.add_map();
            ptr->set_doortag(node.open_);
            ptr->set_nodeid(id);
        }

        m_liftdoor_updated_.clear();
    }

    void setTag(const sim_data_flow::MTransfer &msgs) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);

        m_liftdoor_updated_.clear();
        for (auto &itr : msgs.map()) {
            int node_id = itr.nodeid();

            m_liftdoor_[node_id].open_ = itr.doortag();
            m_liftdoor_updated_.insert(std::make_pair(node_id, m_liftdoor_[node_id]));
        }
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        const float BASE_RANGE = 5;
        if (direction) {
            // shadow (ecal -> webots)
            for (auto &[id, node] : m_liftdoor_updated_) {
                if (node.open_ == true) {
                    node.customdata_f_ptr_->setSFString("open");

                } else {
                    node.customdata_f_ptr_->setSFString("close");
                }
            }
            m_liftdoor_updated_.clear();
        } else {
            // master (webots -> ecal)
            static int spin_cnt = 0;
            spin_cnt++;
            auto position = robot_->getPosition();
            for (auto &[id, itr] : m_liftdoor_) {
                bool door_is_open = false;
                auto p = itr.position;
                if (std::abs(p[0] - position[0]) <= BASE_RANGE &&
                    std::abs(p[1] - position[1]) <= BASE_RANGE) {  // 判断上次是否在车周围
                    itr.customdata_f_ptr_->setSFString("open");
                    door_is_open = true;
                } else {
                    itr.customdata_f_ptr_->setSFString("close");
                    door_is_open = false;
                }

                if (itr.open_ != door_is_open || spin_cnt % 10 == 0) {
                    itr.open_ = door_is_open;
                    m_liftdoor_updated_.insert(std::make_pair(id, itr));
                }
            }
        }
    }

   private:
    void getLiftDoorNode() {
        if (root_ == nullptr)
            return;

        auto children = root_->getField("children");
        auto count = children->getCount();
        for (int i = 0; i < count; ++i) {
            auto node = children->getMFNode(i);
            auto node_type = node->getTypeName();
            if (node_type.compare("LiftDoor") == 0) {
                auto field = node->getField("customData");
                auto manufield = node->getField("manumode");
                WNodeInfo temp(node, field, manufield);
                auto position = node->getPosition();
                memcpy(temp.position, position, sizeof(*position) * 3);
                auto id = node->getId();
                auto pair = std::make_pair(id, temp);
                m_liftdoor_.insert(pair);
                m_liftdoor_updated_.insert(pair);  // 初始化
            }
        }
    }

    Node *root_ = nullptr;
    Node *robot_ = nullptr;
    std::map<uint64_t, WNodeInfo> m_liftdoor_;
    std::map<uint64_t, WNodeInfo> m_liftdoor_updated_;
    int32_t idx = 0;
    bool send_all = false;
    bool direction = false;
};

}  // namespace VNSim