#include <iostream>

#include "ModelDirector.hpp"
#include "Parser/parser.hpp"
#include "tinyxml2.h"

using namespace tinyxml2;

static std::shared_ptr<class ActuatorDirector> ad_ptr{nullptr};

static std::shared_ptr<class SencerDirector> sd_ptr{nullptr};

InputDecoder::InputDecoder() {}

InputDecoder::~InputDecoder() {}

int InputDecoder::loadConfig(const char *path) {
  int ret = 0;
  // std::cout << path << std::endl;
  do {
    XMLDocument doc;
    XMLError error = doc.LoadFile(path);
    bool isOver = false;
    bool isStart = false;
    std::vector<struct SpecialParam> param_list;
    if (error != XMLError::XML_SUCCESS) {
      ret = -1;
      break;
    }
    if (ad_ptr == nullptr) {
      ad_ptr = std::make_shared<class ActuatorDirector>();
    }
    for (XMLElement *ele = doc.RootElement(); ele != nullptr;
         ele = ele->NextSiblingElement()) {
      if (isOver) {
        break;
      }
      // // std::cout << ele->Name() << std::endl;
      for (const XMLAttribute *ch_attr = ele->FirstAttribute();
           ch_attr != nullptr; ch_attr = ch_attr->Next()) {
        // // std::cout << ch_attr->Name() << " " << ch_attr->Value() <<
        // std::endl;
      }
      for (const XMLElement *ch_ele = ele->FirstChildElement();
           ch_ele != nullptr; ch_ele = ch_ele->NextSiblingElement()) {
        // // std::cout << ch_ele->Name() << std::endl;
        std::string name(ch_ele->Name());
        std::string whichFork;
        int length = -1;
        struct SpecialParam param;
        if (name == "DataHeader") {
          isStart = true;
        }
        if (!isStart) {
          continue;
        }
        for (const XMLAttribute *ch_attr = ch_ele->FirstAttribute();
             ch_attr != nullptr; ch_attr = ch_attr->Next()) {
          // // std::cout << ch_attr->Name() << " " << ch_attr->Value() <<
          // std::endl;
          std::string l_name = ch_attr->Name();
          if (l_name == "Length") {
            length = atoi(ch_attr->Value());
          } else {
            param.name = l_name;
            param.value = ch_attr->Value();
            param_list.push_back(param);
            if (param.name == "Function" &&
                (param.value == "P" || param.value == "Z")) {
              whichFork = param.value;
            }
          }
        }
        if (name == "ForkDevice") {
          name += whichFork;
        }
        ad_ptr->create(name.c_str(), param_list, length);
        param_list.clear();
        if (name == "DataTail") {
          isOver = true;
          break;
        }
      }
    }
  } while (0);
  // std::cout << ret << std::endl;
  return ret;
}

int InputDecoder::decodePackage(const struct Package *package) {
  ad_ptr->decode(package->buf, package->len);
  return 0;
}

// int InputDecoder::getHandle(const char* key){
//     int ret = -1;
//     do{
//         if(ad_ptr == nullptr){
//             break;
//         }
//         ret = ad_ptr->get(key);
//     }while(0);
//     return ret;
// }

// int InputDecoder::getValue(int handle, void* value){
//     int ret = 0;
//     std::shared_ptr<class AcuratorModel> ptr = ad_ptr->get(handle);
//     int length = 2;
//     do{
//         if(ptr == nullptr){
//             ret = -1;
//             break;
//         }
//         length = ptr->getLength();
//         memcpy(value, ptr->getValue(), length);
//     }while(0);
//     return ret;
// }
int InputDecoder::getValue(const char *key, void *direct_output_value,
                           double *calculated_output) {
  int ret = 0;
  do {
    if (ad_ptr == nullptr) {
      ret = -1;
      break;
    }
    int handle = ad_ptr->get(key);
    std::shared_ptr<class AcuratorModel> ptr = ad_ptr->get(handle);
    int length = ptr->getLength();
    memcpy(direct_output_value, ptr->getValue(), length);
    if (ptr->isNeedParsed()) {
      ptr->solveValue(calculated_output);
    } else {
      // Not changed;
      *calculated_output = 0;
    }
  } while (0);
  return ret;
}

// int InputDecoder::solveValue(int handle, float input, double* output){
//     int ret = 0;
//     do{
//         if(ad_ptr == nullptr){
//             ret = -1;
//             break;
//         }
//         ad_ptr->solveValue(handle, input, output);
//     }while(0);
//     return ret;
// }

OutputEncoder::OutputEncoder() {}

OutputEncoder::~OutputEncoder() {}

int OutputEncoder::loadConfig(const char *path) {
  int ret = 0;
  // std::cout << path << std::endl;
  do {
    XMLDocument doc;
    XMLError error = doc.LoadFile(path);
    bool isOver = false;
    bool isStart = false;
    std::vector<struct SpecialParam> param_list;
    if (error != XMLError::XML_SUCCESS) {
      ret = -1;
      break;
    }
    if (sd_ptr != nullptr) {
      ret = -2;
      break;
    }
    sd_ptr = std::make_shared<class SencerDirector>();
    for (XMLElement *ele = doc.RootElement(); ele != nullptr;
         ele = ele->NextSiblingElement()) {
      if (isOver) {
        break;
      }
      // // std::cout << ele->Name() << std::endl;
      for (const XMLAttribute *ch_attr = ele->FirstAttribute();
           ch_attr != nullptr; ch_attr = ch_attr->Next()) {
        // // std::cout << ch_attr->Name() << " " << ch_attr->Value() <<
        // std::endl;
      }
      for (const XMLElement *ch_ele = ele->FirstChildElement();
           ch_ele != nullptr; ch_ele = ch_ele->NextSiblingElement()) {
        // std::cout << ch_ele->Name() << std::endl;
        std::string name(ch_ele->Name());
        std::string whichFork;
        int length = -1;
        struct SpecialParam param;
        if (name == "DataHeader") {
          isStart = true;
        }
        if (!isStart) {
          continue;
        }
        for (const XMLAttribute *ch_attr = ch_ele->FirstAttribute();
             ch_attr != nullptr; ch_attr = ch_attr->Next()) {
          // // std::cout << ch_attr->Name() << " " << ch_attr->Value() <<
          // std::endl;
          std::string l_name = ch_attr->Name();
          if (l_name == "Length") {
            length = atoi(ch_attr->Value());
          } else {
            param.name = l_name;
            param.value = ch_attr->Value();
            param_list.push_back(param);
            if (param.name == "Function" &&
                (param.value == "P" || param.value == "Z" ||
                 param.value == "Y" || param.value == "Z")) {
              // TODO: May be give RPMSensor --> M later
              whichFork = param.value;
            }
          }
        }
        if (name == "ForkDisplacementSencer" || name == "Accelerometer" ||
            name == "AngularVelocitySensor") {
          name += whichFork;
        }
        sd_ptr->create(name.c_str(), param_list, length);
        param_list.clear();
        if (name == "DataTail") {
          isOver = true;
          break;
        }
      }
    }
  } while (0);
  return ret;
}

// int OutputEncoder::getHandle(const char* key){
//     int ret = -1;
//     do{
//         if(sd_ptr == nullptr){
//             break;
//         }
//         ret = sd_ptr->get(key);
//     }while(0);
//     return ret;
// }

int OutputEncoder::updateValue(
    const char *key, const std::vector<double> &calculated_input,
    const std::vector<struct UpdateValue *> &direct_input) {
  int ret = 0;
  do {
    if (sd_ptr == nullptr) {
      ret = -1;
      break;
    }
    int handle = sd_ptr->get(key);
    std::shared_ptr<class SensorModel> ptr = sd_ptr->get(handle);
    if (ptr->isNeedParsed()) {
      std::vector<double> calculated_output;
      std::vector<struct UpdateValue> temp;
      std::vector<struct UpdateValue *> transfer_input;
      ptr->solveValue(calculated_input, calculated_output);
      // temp.resize(calculated_output.size());
      // transfer_input.resize(calculated_output.size());
      // for(int i = 0; i < calculated_output.size(); i++){
      //     // int16_t t = calculated_output[i];
      //     // std::cout << calculated_output[i] << std::endl;
      //     // std::cout << t << std::endl;
      //     // temp[i].val = &calculated_output[i];
      //     temp[i].val = &t;
      //     temp[i].subId = i;
      //     temp[i].len = sizeof(double);
      //     transfer_input[i] = &temp[i];
      // }
      // ptr->setValue(transfer_input);
    } else {
      ptr->setValue(direct_input);
    }
  } while (0);
  return ret;
}

// int OutputEncoder::solveValue(int handle, const std::vector<double>& input,
// std::vector<double>& output){
//     int ret = 0;
//     do{
//         if(sd_ptr == nullptr){
//             ret = -1;
//             break;
//         }
//         sd_ptr->solveValue(handle, input, output);
//     }while(0);
//     return ret;
// }

const struct Package *OutputEncoder::encodePackage() {
  const struct Package *pack = nullptr;
  do {
    if (sd_ptr == nullptr) {
      break;
    }
    pack = sd_ptr->get();
  } while (0);
  return pack;
}