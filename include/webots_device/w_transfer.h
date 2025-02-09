/**
 * @file w_transfer.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-06-24
 *
 * @copyright Copyright (c) 2024
 *
 */
// TODO: 用kd树优化
#pragma once

#include <regex>
#include <webots/Node.hpp>
#include <shared_mutex>
#include <nlohmann/json.hpp>

#include "webots_device/w_base.h"
#include "logvn/logvn.h"
#include "sim_data_flow/transfer.pb.h"
#include "VoyerBeltManager.h"

namespace VNSim {
using namespace webots;

typedef struct WTransferNode {
    WTransferNode(Node *node_p = nullptr, Field *rotation_p = nullptr, Field *translation_p = nullptr) {
        node_ptr_ = node_p;
        rotation_f_ptr_ = rotation_p;
        translation_f_ptr_ = translation_p;
    }

    double translation[3] = {0, 0, 0};
    double rotation[4] = {0, 0, 0, 0};

    double tran_set[3] = {0, 0, 0};
    double rota_set[4] = {0, 0, 0, 0};

    double tran_world[3] = {0, 0, 0};
    double tran_world_last[3] = {0, 0, 0};

    Field *rotation_f_ptr_ = nullptr;
    Field *translation_f_ptr_ = nullptr;

    Node *node_ptr_ = nullptr;

    // 暂时只用于叉端雷达随动逻辑
    bool may_need_check_local = false;

    void initWorldPosi(const double *t) {
        for (int i = 0; i < 3; ++i) {
            tran_world[i] = t[i];
            tran_world_last[i] = t[i];
        }
    }
};

class WTransfer : public WBase {
   public:
    WTransfer(std::shared_ptr<VoyerBeltManager> manager = nullptr) : WBase() {
        manager_ = manager;

        root_ = super_->getRoot();

        robot_ = super_->getFromDef("RobotNode");

        idx = 0;
        int tree_depth = 0;
        // printf("base base base TypeName = %s, add = %p, name = %s\n", root_->getBaseTypeName().c_str(), root_);
        // get all node
        getChildNode(root_, tree_depth);

        LOG_INFO("size of tran map %d", m_tanfer_.size());
    }

    void noticeAll() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        send_all = true;
    }

    void getTransfer(sim_data_flow::MTransfer &tanfer_msgs) {
        AutoAtomicLock lock(spin_mutex_);
        static uint64_t seq = 0;
        tanfer_msgs.clear_map();
        tanfer_msgs.set_seq(seq++);
        if (m_updated_trans_.empty()) {
            return;
        }
        for (auto it = m_updated_trans_.begin(); it != m_updated_trans_.end(); ++it) {
            auto ptr = tanfer_msgs.add_map();
            ptr->set_nodeid(it->first);

            ptr->mutable_translation()->set_x(it->second.translation[0]);
            ptr->mutable_translation()->set_y(it->second.translation[1]);
            ptr->mutable_translation()->set_z(it->second.translation[2]);

            ptr->mutable_rotation()->set_x(it->second.rotation[0]);
            ptr->mutable_rotation()->set_y(it->second.rotation[1]);
            ptr->mutable_rotation()->set_z(it->second.rotation[2]);
            ptr->mutable_rotation()->set_w(it->second.rotation[3]);
        }
        m_updated_trans_.clear();
    }

    void setTransfer(const sim_data_flow::MTransfer &msgs) {
        AutoAtomicLock lock(spin_mutex_);

        set_flag = true;
        m_updated_trans_.clear();
        for (int i = 0; i < msgs.map().size(); i++) {
            auto node_id = msgs.map().at(i).nodeid();
            if (m_tanfer_.find(node_id) == m_tanfer_.end()) {
                LOG_INFO("can`t find %d", node_id);
                return;
            }

            m_tanfer_[node_id].tran_set[0] = msgs.map().at(i).translation().x();
            m_tanfer_[node_id].tran_set[1] = msgs.map().at(i).translation().y();
            m_tanfer_[node_id].tran_set[2] = msgs.map().at(i).translation().z();

            m_tanfer_[node_id].rota_set[0] = msgs.map().at(i).rotation().x();
            m_tanfer_[node_id].rota_set[1] = msgs.map().at(i).rotation().y();
            m_tanfer_[node_id].rota_set[2] = msgs.map().at(i).rotation().z();
            m_tanfer_[node_id].rota_set[3] = msgs.map().at(i).rotation().w();
            m_updated_trans_.insert(std::make_pair(node_id, m_tanfer_[node_id]));
        }
    }

    void spin() {
        const float POSI_THRESHOLD = 0.00001;
        const float BASE_RANGE = 10;
        if (set_flag) {
            // 写入(shadow)
            for (auto it = m_updated_trans_.begin(); it != m_updated_trans_.end(); ++it) {
                Field *tran_ptr = it->second.translation_f_ptr_;
                Field *rota_ptr = it->second.rotation_f_ptr_;
                tran_ptr->setSFVec3f(it->second.tran_set);
                rota_ptr->setSFRotation(it->second.rota_set);
            }
            m_updated_trans_.clear();
        } else {
            // 读出(master)
            decltype(m_updated_trans_) tmp_trans;
            const double *robot_tran = robot_->getPosition();
            for (auto it = m_tanfer_.begin(); it != m_tanfer_.end(); ++it) {
                auto last_posi = it->second.tran_world;
                auto curr_posi = it->second.node_ptr_->getPosition();
                // 以下判断逻辑只为优化自动任务时master和shadow间的transform同步
                // 手动移动需手动同步或者手动使用refreshworld按钮
                if (send_all || (std::abs(robot_tran[0] - curr_posi[0]) <= BASE_RANGE &&
                                 std::abs(robot_tran[1] - curr_posi[1]) <= BASE_RANGE)) {  // 在车体周围
                    auto curr_posi = it->second.node_ptr_->getPosition();
                    if (send_all || it->second.may_need_check_local ||
                        std::abs(curr_posi[0] - last_posi[0]) >= POSI_THRESHOLD ||
                        std::abs(curr_posi[1] - last_posi[1]) >= POSI_THRESHOLD ||
                        std::abs(curr_posi[2] - last_posi[2]) >= POSI_THRESHOLD) {  // 世界坐标是否发生变化

                        // 可判断局部坐标是否变化进一步减少数据量(移动Pose的情况)
                        auto curr_tran = it->second.translation_f_ptr_->getSFVec3f();
                        auto curr_rota = it->second.rotation_f_ptr_->getSFRotation();

                        memcpy(it->second.translation, curr_tran, sizeof((*curr_tran)) * 3);
                        memcpy(it->second.rotation, curr_rota, sizeof((*curr_rota)) * 4);
                        memcpy(it->second.tran_world, curr_posi, sizeof(*curr_posi) * 3);
                        auto tmp = *it;
                        tmp_trans.insert(tmp);
                    }
                }
            }
            send_all = false;
            if (!tmp_trans.empty()) {  // 非空则更新，否则等get清理;
                                       //避免卡顿后停止移动最新的pose无法更新
                m_updated_trans_.clear();
                m_updated_trans_ = tmp_trans;
            }
        }
    }

   private:
    void getChildNode(Node *this_ptr, int tree_depth, std::string root_solid_name = "") {
        if (this_ptr == nullptr) {
            return;
        }
        std::string l_root_solid_name = root_solid_name;
        Field *children = this_ptr->getField("children");
        if (children != nullptr) {
            int cnt = children->getCount();
            for (int i = 0; i < cnt; i++) {
                Node *iter = children->getMFNode(i);
                if (manager_ != nullptr) {
                    if ((iter->getTypeName().compare("ConvoyerBelt") == 0) && (manager_ != nullptr)) {
                        manager_->addBelt(iter->getField("name")->getSFString(),
                                          iter->getField("p_groups")->getSFString(), iter);
                    }
                    if (this_ptr->getBaseTypeName().compare("Robot") == 0) {
                        l_root_solid_name = "NOTUSE";
                    }
                    if ((l_root_solid_name == "") && (this_ptr->getField("translation") != nullptr) &&
                        (this_ptr->getBaseTypeName().compare("Solid") == 0)) {
                        if (this_ptr->getParentNode() != nullptr) {
                            if (this_ptr->getParentNode()->getBaseTypeName().compare("Robot") != 0) {
                                l_root_solid_name = this_ptr->getField("name")->getSFString();
                            }
                        } 
                        // else {
                        //     l_root_solid_name = this_ptr->getField("name")->getSFString();
                        //     printf("init l_root_solid_name = \n");
                            
                        // }
                    }
                    else{
                        
                        if((iter->getField("translation") != nullptr) &&
                        (iter->getBaseTypeName().compare("Solid") == 0) && (tree_depth == 0)) {
                            if(iter->getField("name") != nullptr)
                                l_root_solid_name = iter->getField("name")->getSFString();
                        }
                    }
                }
                getChildNode(iter, tree_depth + 1, l_root_solid_name);
            }
        }

        if (this_ptr->getTypeName().compare("HingeJoint") == 0) {
            LOG_INFO("there is HingeJoint");
            Field *iter = this_ptr->getField("endPoint");
            Node *endPoint = iter->getSFNode();
            getChildNode(endPoint, tree_depth + 1,  l_root_solid_name);

        } else if (this_ptr->getTypeName().compare("SliderJoint") == 0) {
            LOG_INFO("there is SliderJoint");
            Field *iter = this_ptr->getField("endPoint"); 
            Node *endPoint = iter->getSFNode();
            getChildNode(endPoint, tree_depth + 1,l_root_solid_name);
        } else if (this_ptr->getTypeName().compare("Hinge2Joint") == 0) {
            LOG_INFO("there is Hinge2Joint");
            Field *iter = this_ptr->getField("endPoint");
            Node *endPoint = iter->getSFNode();
            getChildNode(endPoint, tree_depth + 1, l_root_solid_name);
        }

        Field *rotation_ptr_ = this_ptr->getField("rotation");
        Field *translation_ptr_ = this_ptr->getField("translation");

        if (rotation_ptr_ != nullptr && translation_ptr_ != nullptr) {
            auto node_type = this_ptr->getTypeName();           // 用于区分自定义的ProtoType
            auto node_base_type = this_ptr->getBaseTypeName();  // 只区分基本的类型
            auto parent_node = this_ptr->getParentNode();

            if (node_type.compare("Robot") != 0) {
                WTransferNode tran(this_ptr, rotation_ptr_, translation_ptr_);
                tran.initWorldPosi(this_ptr->getPosition());

                if (node_base_type.compare("Lidar") == 0)
                    tran.may_need_check_local = true;
                else
                    tran.may_need_check_local = false;

                auto tmp_pair = std::make_pair(idx++, tran);

                m_tanfer_.insert(tmp_pair);
                m_updated_trans_.insert(tmp_pair);  // 初始化更新
                LOG_INFO("creat tran %d", idx);
            }
            std::regex pattern("Pallets_[a-zA-Z0-9]+");
            bool IsPallet = false;
            if (manager_ != nullptr) {
                auto base_type_name = this_ptr->getBaseTypeName();  // 用于区分自定义的ProtoType
                if (std::regex_match(parent_node->getDef(), pattern) && (base_type_name.compare("Solid") == 0)) {
                    manager_->initPallets(parent_node->getDef(), this_ptr);
                    IsPallet = true;
                }
                Field *name_field = this_ptr->getField("name");
                if (name_field != nullptr) {
                    // printf("NODE = %p ,name_field = %s --> l_root_solid_name = %s\n", this_ptr, name_field->getSFString().c_str(),
                    //        l_root_solid_name.c_str());
                    if (!IsPallet && (name_field->getSFString().compare(l_root_solid_name)) == 0) {
                        // printf("find addPossibleNode %s\n", name_field->getSFString().c_str());
                        manager_->addPossibleNode(this_ptr);
                    }
                }
            }
        }
    }

    Node *root_ = nullptr;
    Node *robot_ = nullptr;
    std::map<int32_t, WTransferNode> m_tanfer_;
    // master for pub,shadow for sub
    std::map<int32_t, WTransferNode> m_updated_trans_;
    // idx specify node id
    int32_t idx = 0;
    bool send_all = false;
    bool set_flag = false;
    std::shared_ptr<VoyerBeltManager> manager_;
};

}  // namespace VNSim
