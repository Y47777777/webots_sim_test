#ifndef __SINGLETON_SOLID_H__
#define __SINGLETON_SOLID_H__
#include <list>
#include <webots/Node.hpp>
#include "singleton/singleton.h"
namespace VNSim{
class PossibleSolidData:public Singleton<PossibleSolidData> {
    public:
        //friend class Singleton<PossibleSolidData>;
        PossibleSolidData();
        ~PossibleSolidData();
    private:
        std::list<webots::Node*> node_list_;
    public:
        void addNode(webots::Node* node);
        const std::list<webots::Node*>* getNode();
};
}
#endif