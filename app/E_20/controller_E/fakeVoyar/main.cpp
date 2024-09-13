#include <thread>
#include "webots_device/w_robot_lidar.h"
#include "ecal_wrapper/ecal_wrapper.h"
#include "sim_data_flow/voyerbelt_msg.pb.h"

using namespace VNSim;

const char * report_topic = "Goods/Events";
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;

int main(int argc, char *argv[]){
    std::shared_ptr<Voyar> test_ptr = nullptr;
    std::shared_ptr<EcalWrapper> ecal_ptr_ = nullptr;
    bool flag = true;
    std::string name = "";
    std::string Sensor_name = "";
    int last_send_event = -1;
    int temp_last_send_event = -1;
    // if(argc < 2){
    //     name = "testSensor_NoName";
    //     Sensor_name = "testSensorDevice_NoName";
    // }
    test_ptr = std::make_shared<Voyar>(name,Sensor_name);
    ecal_ptr_ = EcalWrapper::getInstance(name);
    ecal_ptr_->addEcal(report_topic);
    while(flag){
        test_ptr->spin();
        temp_last_send_event = test_ptr->goodsEvent();
        if((temp_last_send_event != -1) && (temp_last_send_event != last_send_event)){
            // TODO: ecal wrapper
            last_send_event = temp_last_send_event;
            sim_data_flow::StateInfo payload;
            payload.set_belt_name(test_ptr->Robot::getName());
            payload.set_state(last_send_event);
            uint8_t buf[payload.ByteSize()];
            payload.SerializePartialToArray(buf, payload.ByteSize());
            ecal_ptr_->send(report_topic, buf, payload.ByteSize());
            std::cout << "send event --> " << test_ptr->goodsEvent() << std::endl;
        }
    }
    return 0;
}