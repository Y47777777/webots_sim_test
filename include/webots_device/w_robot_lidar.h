/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:08
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 10:40:23
 * @FilePath: /webots_ctrl/include/webots_device/w_base.h
 * @Description:
 *                  webots 接口base
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <shared_mutex>
#include "logvn/logvn.h"
#include "ecal_wrapper/ecal_wrapper.h"
#include "time/time.h"

namespace VNSim {
using namespace webots;

class WRobot: public Robot{
    public:
        // TEST
        WRobot(std::string name):Robot(){
            robot_name_ = name;
        }
        ~WRobot(){}
        std::string getName(){return robot_name_;};
    private:
        //static std::shared_ptr<Timer> instance_ptr_;
        std::string robot_name_;
};


enum EVENTS{
    NOT_EVENTS = -1,
    HAVE_GOODS = 0,
    NO_GOODS = 1
};
Timer t;
class Voyar : public WRobot {
   public:
   /**
    * @brief Get the Supervisor Instance object
    * 
    * @return WSupervisor* 
    */
    Voyar(std::string robot_name, std::string name):WRobot(robot_name){
        printf("Name = %s\n", this->Robot::getName().c_str());
        device = this->getDistanceSensor(std::string(this->Robot::getName() + "_D"));
        sensor_name_ = name;
        if(device){
            device->enable(100);
        }
        //robot_name_ = this->getName();
    }

    ~Voyar(){}

    void spin(){
        this->step(this->getBasicTimeStep());
        if(device){
            //t.restart();
            double curr_val = device->getValue();
            //uint32_t diff = t.elapsed<std::chrono::milliseconds>();
            //printf("time elpsted = %d\n", diff);
            //std::cout << "Robot = " << this->getName() << ", sensor = " << sensor_name_ << "read value = " << device->getValue() << std::endl;
            VoyarEvent_ = curr_val < 2 ? HAVE_GOODS : NO_GOODS;
        }
    }

    int goodsEvent(){
        return VoyarEvent_;
    }

    private:
        webots::DistanceSensor *device{nullptr};
        std::string sensor_name_{""};
        int VoyarEvent_{NOT_EVENTS};
        
};  
}  // namespace VNSim