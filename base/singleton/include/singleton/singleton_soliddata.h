#ifndef __SINGLETON_SOLID_H__
#define __SINGLETON_SOLID_H__
#include <list>
#include <shared_mutex>
#include <webots/Node.hpp>
#include "singleton/singleton.h"
namespace VNSim{
class PossibleSolidData:public Singleton<PossibleSolidData> {
    public:
        friend class Singleton<PossibleSolidData>;
        PossibleSolidData();
        ~PossibleSolidData();
    private:
        std::list<webots::Node*> node_list_;
        std::shared_mutex rw_mutex_;
    public:
        void addNode(webots::Node* node);
        int getNode(std::list<webots::Node*>& node_list);
};
}
#endif