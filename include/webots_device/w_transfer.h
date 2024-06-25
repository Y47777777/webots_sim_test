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
#pragma once

#include <webots/Node.hpp>
#include <shared_mutex>
#include <nlohmann/json.hpp>

#include "webots_device/w_base.h"
#include "logvn/logvn.h"
#include "sim_data_flow/transfer.pb.h"

namespace VNSim {
using namespace webots;

typedef struct STransfer {
    STransfer(Field *rotation_p = nullptr, Field *translation_p = nullptr) {
        rotation_f_ptr_ = rotation_p;
        translation_f_ptr_ = translation_p;
    }

    double translation[3] = {0, 0, 0};
    double rotation[4] = {0, 0, 0, 0};

    double tran_set[3] = {0, 0, 0};
    double rota_set[4] = {0, 0, 0, 0};

    Field *rotation_f_ptr_ = nullptr;
    Field *translation_f_ptr_ = nullptr;
};

class WTransfer : public WBase {
   public:
    WTransfer() : WBase() {
        root_ = super_->getRoot();

        // get all node
        getChildNode(root_);

        LOG_INFO("size of tran map %d", m_tanfer_.size());
    }

    void getTransfer(sim_data_flow::MTransfer &tanfer_msgs) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);

        tanfer_msgs.clear_map();
        for (auto it = m_tanfer_.begin(); it != m_tanfer_.end(); ++it) {
            auto ptr = tanfer_msgs.add_map();
            ptr->set_name(it->first);

            ptr->mutable_translation()->set_x(it->second.translation[0]);
            ptr->mutable_translation()->set_y(it->second.translation[1]);
            ptr->mutable_translation()->set_z(it->second.translation[2]);

            ptr->mutable_rotation()->set_x(it->second.rotation[0]);
            ptr->mutable_rotation()->set_y(it->second.rotation[1]);
            ptr->mutable_rotation()->set_z(it->second.rotation[2]);
            ptr->mutable_rotation()->set_w(it->second.rotation[3]);

        }
    }

    void setTransfer(const sim_data_flow::MTransfer &msgs) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);

        set_flag = true;
        for (int i = 0; i < msgs.map().size(); i++) {
            std::string name = msgs.map().at(i).name();
            if (m_tanfer_.find(name) == m_tanfer_.end()) {
                LOG_INFO("can`t find %s", name.c_str());
                return;
            }

            m_tanfer_[name].tran_set[0] = msgs.map().at(i).translation().x();
            m_tanfer_[name].tran_set[1] = msgs.map().at(i).translation().y();
            m_tanfer_[name].tran_set[2] = msgs.map().at(i).translation().z();

            m_tanfer_[name].rota_set[0] = msgs.map().at(i).rotation().x();
            m_tanfer_[name].rota_set[1] = msgs.map().at(i).rotation().y();
            m_tanfer_[name].rota_set[2] = msgs.map().at(i).rotation().z();
            m_tanfer_[name].rota_set[3] = msgs.map().at(i).rotation().w();
        }
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        if (set_flag) {
            // 写入(shadow)
            for (auto it = m_tanfer_.begin(); it != m_tanfer_.end(); ++it) {
                Field *tran_ptr = it->second.translation_f_ptr_;
                Field *rota_ptr = it->second.rotation_f_ptr_;

                tran_ptr->setSFVec3f(it->second.tran_set);
                rota_ptr->setSFRotation(it->second.rota_set);
            }
        } else {
            // 读出(master)
            for (auto it = m_tanfer_.begin(); it != m_tanfer_.end(); ++it) {
                const double *tran =
                    it->second.translation_f_ptr_->getSFVec3f();
                const double *rota =
                    it->second.rotation_f_ptr_->getSFRotation();

                memcpy(it->second.translation, tran, sizeof((*tran)) * 3);
                memcpy(it->second.rotation, rota, sizeof((*rota)) * 4);
            }
        }
    }

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

        Field *rotation_ptr_ = this_ptr->getField("rotation");
        Field *translation_ptr_ = this_ptr->getField("translation");

        if (rotation_ptr_ != nullptr && translation_ptr_ != nullptr) {
            auto name_field = this_ptr->getField("name");
            if (name_field == nullptr) {
                return;
            }

            std::string name = name_field->getSFString();
            if (name != "Robot") {
                STransfer tran(rotation_ptr_, translation_ptr_);
                m_tanfer_.insert(std::pair<std::string, STransfer>(name, tran));
                LOG_INFO("creat tran %s", name.c_str());
            }
        }
    }

    Node *root_ = nullptr;
    std::map<std::string, STransfer> m_tanfer_;
    bool set_flag = false;
};

}  // namespace VNSim
