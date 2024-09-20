#include <memory>
#include "singleton/singleton_soliddata.h"

using namespace VNSim;

PossibleSolidData::PossibleSolidData(){}

PossibleSolidData::~PossibleSolidData(){}

void PossibleSolidData::addNode(webots::Node* node){
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    node_list_.push_back(node);
}

int PossibleSolidData::getNode(std::list<webots::Node*>& node_list){
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    for(auto& it:node_list_)
        node_list.push_back(it);
    return 0;
}
