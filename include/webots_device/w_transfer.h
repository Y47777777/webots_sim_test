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

    nlohmann::json getTransfer() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        nlohmann::json msg;

        for (auto it = m_tanfer_.begin(); it != m_tanfer_.end(); ++it) {
            double *translation = it->second.translation;
            double *rotation = it->second.rotation;

            msg[it->first]["translation"]["x"] = translation[0];
            msg[it->first]["translation"]["y"] = translation[1];
            msg[it->first]["translation"]["z"] = translation[2];

            msg[it->first]["rotation"]["x"] = rotation[0];
            msg[it->first]["rotation"]["y"] = rotation[1];
            msg[it->first]["rotation"]["z"] = rotation[2];
            msg[it->first]["rotation"]["w"] = rotation[3];
        }

        return msg;
    }

    void setTransfer(nlohmann::json msg) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
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

                memcpy(it->second.translation, tran,
                       sizeof((*it->second.translation)) * 3);
                memcpy(it->second.rotation, rota,
                       sizeof((*it->second.rotation)) * 4);
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
