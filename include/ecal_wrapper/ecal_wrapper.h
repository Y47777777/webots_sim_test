/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-03 16:28:14
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 16:23:24
 * @FilePath: /webots_ctrl/include/ecal_wrapper/ecal_wrapper.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#ifndef __ECAL_WRAPPER__H__
#define __ECAL_WRAPPER__H__
#include <map>
#include "ecal/ecal.h"

// TODO: This is a class to manage, ecal
// May be try use class template to solve
namespace VNSim {
class EcalWrapper {
   public:
    /**
     * @brief Get the Instance object
     *
     * @return std::shared_ptr<Timer>
     */
    static std::shared_ptr<EcalWrapper> getInstance(
        std::string name = "wrapper") {
        if (instance_ptr_ == nullptr) {
            instance_ptr_ = std::make_shared<EcalWrapper>();
            const char *str = name.c_str();
            eCAL::Initialize(0, 0, str);
        }
        return instance_ptr_;
    }

    void exit() {
        // TODO: exit 正常退出
    }
    EcalWrapper() {}
    ~EcalWrapper() {
        // TODO: exit 正常退出
    }

    int addEcal(const char *name,
                const std::function<void(const char *,
                                         const eCAL::SReceiveCallbackData *)>
                    &callback) {
        if (sub_map_.find(name) == sub_map_.end()) {
            sub_map_[name] = std::make_unique<eCAL::CSubscriber>(name);
            sub_map_[name]->AddReceiveCallback(callback);
        }
        return 0;
    }
    int addEcal(const char *name) {
        pub_map_[name] = std::make_unique<eCAL::CPublisher>(name);
        return 0;
    }
    int send(const char *name, const uint8_t *data, int bytes) {
        size_t send_bytes = pub_map_[name]->Send(data, bytes);
        return (send_bytes != bytes) ? -1 : 0;
    }

   private:
    static std::shared_ptr<EcalWrapper> instance_ptr_;

   private:
    std::map<std::string, std::unique_ptr<eCAL::CPublisher>> pub_map_;
    std::map<std::string, std::unique_ptr<eCAL::CSubscriber>> sub_map_;
};
}  // namespace VNSim

#endif