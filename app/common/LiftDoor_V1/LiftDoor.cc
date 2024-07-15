#include <webots/Supervisor.hpp>
#include <iostream>
#include <vector>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
using namespace webots;

int main(){
    Robot temp;
    auto door = temp.getMotor("linear motor");
    door->setPosition(INFINITY);
    
    while(temp.step(8) != -1){
        auto cust = temp.getCustomData();
        if(cust.compare("close") == 0){
            door->setVelocity(-1);
        }else if(cust.compare("open") == 0){
            door->setVelocity(1);
        }
    }
    
    return 0;
}