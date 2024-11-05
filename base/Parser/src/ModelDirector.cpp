#include <cstring>
#include <iostream>

#include "ModelDirector.hpp"
#include "utils.hpp"

ActuatorDirector::ActuatorDirector() {
    total_ = 0;
}

ActuatorDirector::~ActuatorDirector() {
    std::vector<std::shared_ptr<class AcuratorModel>> empty_package;
    std::map<std::string, int> empty_supportss;
    package_set_.swap(empty_package);
    support_handle_.swap(empty_supportss);
}

void ActuatorDirector::create(const char *name, const char *function,
                              const std::vector<struct SpecialParam> &param,
                              int length, bool is32bits) {
    std::shared_ptr<class AcuratorModel> ptr = nullptr;
    bool isSupportModified = true;
    if (strcmp(name, "MoveDevice") == 0) {
        ptr = std::make_shared<MoveDevice>();
        // std::cout << "create MoveDevice = " << name << std::endl;
    } else if (strcmp(name, "SteeringDevice") == 0) {
        ptr = std::make_shared<SteeringDevice>();
        // std::cout << "create SteeringDevice = " << name << std::endl;
    } else if (strcmp(name, "ForkDevice") == 0) {
        ptr = std::make_shared<ForkDevice>();
        // std::cout << "create ForkDevice = " << name << std::endl;
    } else if (strcmp(name, "LiftDevice") == 0) {
        ptr = std::make_shared<LiftDevice>();
        // std::cout << "create LiftDevice = " << name << std::endl;
    } else if (strcmp(name, "SwitchActuator") == 0) {
        ptr = std::make_shared<SwitchActuator>();
        // std::cout << "create SwitchActuator = " << name << std::endl;
    } else if (strcmp(name, "DataIndex") == 0) {
        ptr = std::make_shared<ADataIndex>();
        // std::cout << "create ADataIndex = " << name << std::endl;
    } else if (strcmp(name, "NullActuator") == 0) {
        ptr = std::make_shared<NullActuator>();
        // std::cout << "create NullActuator = " << name << std::endl;
        isSupportModified = false;
    } else if (strcmp(name, "MCUDataIndexReturn") == 0) {
        ptr = std::make_shared<MCUDataIndexReturn>();
        // std::cout << "create MCUDataIndexReturn = " << name << std::endl;
    }
    // else if(strcmp(name, "ValveControleDevice") == 0){
    //     if(length == -1){
    //         return;
    //     }
    //     ptr = std::make_shared<ValveControleDevice>();
    //     // std::cout << "create ValveControleDevice = " << name << std::endl;
    // }else if(strcmp(name, "SerialDataActuator") == 0){
    //     if(length == -1){
    //         return;
    //     }
    //     ptr = std::make_shared<SerialDataActuator>();
    //     // std::cout << "create SerialDataActuator = " << name << std::endl;
    // }else if(strcmp(name, "GPIOSwitchActuator") == 0){
    //     if(length == -1){
    //         return;
    //     }
    //     ptr = std::make_shared<GPIOSwitchActuator>();
    //     // std::cout << "create GPIOSwitchActuator = " << name << std::endl;
    // }else if(strcmp(name, "CanToWifiActuator") == 0){
    //     if(length == -1){
    //         return;
    //     }
    //     ptr = std::make_shared<CanToWifiActuator>();
    //     // std::cout << "create CanToWifiActuator = " << name << std::endl;
    // }
    else {
        ptr = std::make_shared<ADataIgnore>();
        // std::cout << "create ADataIgnore = " << name << std::endl;
        isSupportModified = false;
    }
    // std::cout << "length = " << length << std::endl;
    int endP = 0;
    if (!package_set_.empty()) {
        endP = package_set_.back()->getEndP() + 1;
    }
    // std::cout << endP << std::endl;
    ptr->config(param, endP, length, is32bits);
    package_set_.push_back(ptr);
    total_++;
    // package_set_.at(package_set_.size() - 1)->Print();
    std::string combined_key = std::string(name) + std::string(function);
    if (isSupportModified) {
        support_handle_[combined_key] = total_ - 1;
        // std::cout << "combined_key = " << combined_key
        //           << ", handle = " << support_handle_[combined_key]
        //           << ", start = " << endP
        //           << ", end = " << endP + ptr->getLength() - 1 << std::endl;
    }
    return;
}

void ActuatorDirector::decode(uint8_t *buf, int length) {
    for (auto &ele : package_set_) { ele->setValue(buf); }
}

int ActuatorDirector::get(const char *name, const char *func_key) {
    std::string combined_key = std::string(name) + std::string(func_key);
    return support_handle_[combined_key];
}

std::shared_ptr<class AcuratorModel> ActuatorDirector::get(int handle) {
    return package_set_.at(handle);
}

// void ActuatorDirector::solveValue(int handle, float input, double* output){
//     return package_set_.at(handle)->solveValue(input, output);
// }

SencerDirector::SencerDirector() {
    total_ = 0;
    CRC_Handle_ = 0;
    package_.buf = nullptr;
    package_.len = 0;
}

SencerDirector::~SencerDirector() {
    if (package_.buf != nullptr) {
        delete[] package_.buf;
    }
    std::vector<std::shared_ptr<class SensorModel>> empty_package;
    std::map<std::string, int> empty_supportss;
    package_set_.swap(empty_package);
    support_handle_.swap(empty_supportss);
}

void SencerDirector::create(const char *name, const char *function,
                            const std::vector<struct SpecialParam> &param,
                            int length, bool is32bits) {
    std::shared_ptr<class SensorModel> ptr = nullptr;
    bool isSupportModified = true;
    if (strcmp(name, "WheelCoder") == 0) {
        ptr = std::make_shared<WheelCoder>();
        // std::cout << "create WheelCoder = " << name << std::endl;
    } else if (strcmp(name, "BatterySencer") == 0) {
        ptr = std::make_shared<BatterySencer>();
        // std::cout << "create BatterySencer = " << name << std::endl;
    } else if (strcmp(name, "IncrementalSteeringCoder") == 0) {
        ptr = std::make_shared<IncrementalSteeringCoder>();
        // std::cout << "create IncrementalSteeringCoder = " << name <<
        // std::endl;
    }else if (strcmp(name, "Gyroscope") == 0) {
        ptr = std::make_shared<Gyroscope>();
        // std::cout << "create Gyroscope = " << name << std::endl;
    } else if (strcmp(name, "ForkDisplacementSencer") == 0 || 
    strcmp(name, "DisplacementSencer") == 0) {
        ptr = std::make_shared<ForkDisplacementSencer>();
        // std::cout << "create ForkDisplacementSencer = " << name <<
        // std::endl;
    } else if (strcmp(name, "HeightCoder") == 0) {
        ptr = std::make_shared<HeightCoder>();
        // std::cout << "create HeightCoder = " << name << std::endl;
    } else if (strcmp(name, "DataIndex") == 0) {
        ptr = std::make_shared<SDataIndex>();
        // std::cout << "create SDataIndex = " << name << std::endl;
    } else if (strcmp(name, "DataIndexReturn") == 0) {
        ptr = std::make_shared<DataIndexReturn>();
        // std::cout << "create DataIndexReturn = " << name << std::endl;
    } else if (strcmp(name, "NullSencer") == 0) {
        ptr = std::make_shared<NullSencer>();
        isSupportModified = false;
        // std::cout << "create NullSencer = " << name << std::endl;
    } else if (strcmp(name, "DataHeader") == 0) {
        ptr = std::make_shared<DataHeader>();
        // std::cout << "create DataHeader = " << name << std::endl;
    } else if (strcmp(name, "DataCRC") == 0) {
        ptr = std::make_shared<DataCRC>();
        // std::cout << "create DataCRC = " << name << std::endl;
    } else if (strcmp(name, "DataTail") == 0) {
        ptr = std::make_shared<DataTail>();
        // std::cout << "create DataTail = " << name << std::endl;
    } else if (strcmp(name, "ErrorCode") == 0) {
        ptr = std::make_shared<ErrorCode>();
        // std::cout << "create ErrorCode = " << name << std::endl;
    } else if (strcmp(name, "VelocityControlLevel") == 0) {
        ptr = std::make_shared<VelocityControlLevel>();
        // std::cout << "create VelocityControlLevel = " << name << std::endl;
    } else if (strcmp(name, "RPMSensor") == 0) {
        ptr = std::make_shared<RPMSensor>();
        // std::cout << "create RPMSensor = " << name << std::endl;
    } else if (strcmp(name, "SwitchSencer") == 0) {
        ptr = std::make_shared<SwitchSencer>();
        // std::cout << "create SwitchSencer = " << name << std::endl;
    } else if (strcmp(name, "Accelerometer") == 0) {
        ptr = std::make_shared<Accelerometer>();
        // std::cout << "create AccelerometerX = " << name << std::endl;
    } else if (strcmp(name, "AngularVelocitySensor") == 0) {
        ptr = std::make_shared<AngularVelocitySensor>();
        // std::cout << "create AngularVelocitySensorX = " << name << std::endl;
    } else if (strcmp(name, "HydraulicPressureSensor") == 0) {
        ptr = std::make_shared<HydraulicPressureSensor>();
        // std::cout << "create HydraulicPressureSensor = " << name <<
        // std::endl;
    } else if (strcmp(name, "ElePerceptionCameraDistance") == 0) {
        ptr = std::make_shared<ElePerceptionCameraDistance>();
        // std::cout << "create ElePerceptionCameraDistance = " << name <<
        // std::endl;
    } else {
        ptr = std::make_shared<SDataIgnore>();
        // std::cout << "create SDataIgnore = " << name << std::endl;
        isSupportModified = false;
    }
    // // std::cout << "length = " << length << std::endl;
    int endP = 0;
    if (!package_set_.empty()) {
        endP = package_set_.back()->getEndP() + 1;
    }
    ptr->config(param, endP, length, is32bits);
    package_set_.push_back(ptr);
    total_++;
    package_.len += ptr->getLength();
    // package_set_.at(package_set_.size() - 1)->Print();
    if (strcmp(name, "DataCRC") == 0) {
        CRC_Handle_ = total_ - 1;
    }
    std::string combined_key = std::string(name) + std::string(function);
    if (isSupportModified) {
        support_handle_[combined_key] = total_ - 1;
        // std::cout << "combined_key = " << combined_key
        //           << ", handle = " << support_handle_[combined_key]
        //           << ", start = " << endP
        //           << ", end = " << endP + ptr->getLength() - 1 << std::endl;
    }
    //// std::cout << "TotalBytes = " << package_.len << std::endl;
    return;
}

void SencerDirector::encode(int handle,
                            std::vector<struct UpdateValue *> &val_list) {
    std::shared_ptr<class SensorModel> ptr = package_set_.at(handle);
    package_set_.at(handle)->setValue(val_list);
}

int SencerDirector::get(const char *name, const char *function) {
    std::string combined_key = std::string(name) + std::string(function);
    return support_handle_[combined_key];
}

std::shared_ptr<class SensorModel> SencerDirector::get(int handle) {
    return package_set_.at(handle);
}

// void SencerDirector::solveValue(int handle, const std::vector<double>& input,
// std::vector<double>& output){
//     package_set_.at(handle)->solveValue(input, output);
// }

const struct Package *SencerDirector::get() {
    int s_p = 0;
    int e_p = 0;
    int s_to_e = 0;
    int crc_s_p = package_set_.at(CRC_Handle_)->getStartP();
    int crc_s_to_e = package_set_.at(CRC_Handle_)->getLength();
    int header_s_to_e = package_set_.front()->getLength();
    int tail_s_to_e = package_set_.back()->getLength();
    struct UpdateValue crc_val;
    std::vector<struct UpdateValue *> crc_info;
    int16_t crc;
    if (package_.buf == nullptr) {
        package_.buf = new uint8_t[package_.len];
    }
    for (auto &it : package_set_) {
        s_p = it->getStartP();
        e_p = it->getEndP();
        s_to_e = it->getLength();
        memcpy(package_.buf + s_p, it->getbuf(), s_to_e);
    }
    crc = CalCRC(package_.buf, 0, package_.len - tail_s_to_e - crc_s_to_e,
                 0xffff, 0x8408);
    // printf("package_.len = %d, tail_s_to_e = %d, crc_s_to_e = %d\n",
    // package_.len, tail_s_to_e, crc_s_to_e); printf("size = %d\n",
    // package_.len
    // - tail_s_to_e - crc_s_to_e);
    crc_val.val = &crc;
    crc_val.subId = 0;
    crc_info.push_back(&crc_val);
    package_set_.at(CRC_Handle_)->setValue(crc_info);
    memcpy(package_.buf + crc_s_p, package_set_.at(CRC_Handle_)->getbuf(),
           crc_s_to_e);
    return &package_;
}