#ifndef __ECAL_WRAPPER__H__
#define __ECAL_WRAPPER__H__
#include <map>
#include "ecal/ecal.h"

// TODO: This is a class to manage, ecal
// May be try use class template to solve
namespace VNSim {
class EcalWrapper {
   protected:
    std::map<std::string, std::unique_ptr<eCAL::CPublisher>> pub_map_;
    std::map<std::string, std::unique_ptr<eCAL::CSubscriber>> sub_map_;

   public:
    EcalWrapper() {}
    ~EcalWrapper() {}
    int init(bool onInit, const char *name) {
        if (onInit)
            eCAL::Initialize(0, 0, name);
        return 0;
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
};
}  // namespace VNSim

#endif