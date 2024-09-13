#include "singleton/singleton_soliddata.h"

using namespace VNSim;

PossibleSolidData::PossibleSolidData(){}

PossibleSolidData::~PossibleSolidData(){}

void PossibleSolidData::addNode(webots::Node* node){printf("%s --> possible solid\n", __FUNCTION__);node_list_.push_back(node);}

const std::list<webots::Node*>* PossibleSolidData::getNode(){return &node_list_;}
