#include <webots/Supervisor.hpp>
#include <iostream>
#include <vector>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <string>
using namespace webots;

int main(){
    // Robot temp;
    // auto door = temp.getMotor("linear motor");
    // door->setPosition(INFINITY);
    
    // while(temp.step(8) != -1){
    //     auto cust = temp.getCustomData();
    //     if(cust.compare("close") == 0){
    //         door->setVelocity(-1);
    //     }else if(cust.compare("open") == 0){
    //         door->setVelocity(1);
    //     }
    // }
    auto super = Supervisor::getSupervisorInstance();
    int n = 0;
    std::string operate_type = "open";
    while(super->step(8)!=-1){
        auto node = super->getFromDef("liftdoor");
        if(node)
        {
            auto field = node->getField("customData");
            if(field){
              field->setSFString(operate_type);
            }
        }
        if(n++ == 9){
          operate_type = "close";
        }else if(n == 19){
          operate_type = "open";
          n = 0;
        }
    }
    return 0;
}