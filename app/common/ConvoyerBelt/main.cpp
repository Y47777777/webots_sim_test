#include <thread>
#include "webots_device/w_robot_conveyor.h"
#include "ecal_wrapper/ecal_wrapper.h"
#include "sim_data_flow/voyerbelt_msg.pb.h"

using namespace VNSim;

std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;

int main(int argc, char *argv[]){
    std::shared_ptr<ConvoyerBelt> test_ptr = nullptr;
    std::shared_ptr<EcalWrapper> ecal_ptr_ = nullptr;
    bool flag = true;
    bool onStart = true;
    std::string name = "";
    std::string Sensor_name = "";
    std::string cmd_topic = "";
    std::string report_topic = "";
    int last_send_event = -1;
    int temp_last_send_event = -1;
    // if(argc < 2){
    //     name = "testSensor_NoName";
    //     Sensor_name = "testSensorDevice_NoName";
    // }
    test_ptr = std::make_shared<ConvoyerBelt>(name,Sensor_name);
    cmd_topic = test_ptr->Robot::getName() + "/CmdInfo";
    report_topic = test_ptr->Robot::getName() + "/Goods/Events";
    //printf("cmd_topic = %s\n", cmd_topic.c_str());
    ecal_ptr_ = EcalWrapper::getInstance(name);
    ecal_ptr_->addEcal(report_topic.c_str());
    ecal_ptr_->addEcal(cmd_topic.c_str(), std::bind([&](const char *topic_name,
                        const eCAL::SReceiveCallbackData *data){
                            sim_data_flow::CmdInfo payload;
                            payload.ParseFromArray(data->buf, data->size);    
                            uint32_t cmd = payload.cmd();
                            //printf("I HAVE MESSAGE !!!!!!!!, %s\n", test_ptr->Robot::getName().c_str());
                            if(cmd == 0){
                                //printf("Receive STOP CMD, %s\n", test_ptr->Robot::getName().c_str());
                                onStart = false;
                            }
                        },std::placeholders::_1, std::placeholders::_2));
    // wait for ecal initialization....
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    while(flag){
        test_ptr->spin();
        temp_last_send_event = test_ptr->goodsEvent();
        if(((temp_last_send_event != -1) && (temp_last_send_event != last_send_event)) || onStart){
            // TODO: ecal wrapper
            last_send_event = temp_last_send_event;
            sim_data_flow::StateInfo payload;
            payload.set_belt_name(test_ptr->Robot::getName());
            payload.set_state(last_send_event);
            if(onStart)
                payload.set_who(0);
            else
                payload.set_who(1);
            uint8_t buf[payload.ByteSize()];
            payload.SerializePartialToArray(buf, payload.ByteSize());
            ecal_ptr_->send(report_topic.c_str(), buf, payload.ByteSize());
        }
    }
    return 0;
}