#include <iostream>
#include <stdarg.h>
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
                    // // std::cout << ch_attr->Name() << " " <<
                    // ch_attr->Value() << std::endl;
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

int InputDecoder::getValue(const char *key, double *output) {
    int ret = 0;
    do {
        if (ad_ptr == nullptr) {
            ret = -1;
            break;
        }
        int handle = ad_ptr->get(key);
        std::shared_ptr<class AcuratorModel> ptr = ad_ptr->get(handle);
        if (ptr->isNeedParsed()) {
            ptr->solveValue(output);
        } else {
            ret = -2;
        }
    } while (0);
    return ret;
}

int InputDecoder::getValue2(const char *key, void *output, int size) {
    int ret = 0;
    do {
        if (ad_ptr == nullptr) {
            ret = -1;
            break;
        }
        int handle = ad_ptr->get(key);
        std::shared_ptr<class AcuratorModel> ptr = ad_ptr->get(handle);
        int length = ptr->getLength();
        memcpy(output, ptr->getValue(), length);
    } while (0);
    return ret;
}

int InputDecoder::getSwitchValue(const char *key, int bits, bool *output) {
    int ret = 0;
    do {
        if (ad_ptr == nullptr) {
            ret = -1;
            break;
        }
        int handle = ad_ptr->get(key);
        std::shared_ptr<class AcuratorModel> ptr = ad_ptr->get(handle);
        const uint8_t *header = (const uint8_t *) ptr->getValue();
        int length = ptr->getLength();
        int bytes_index = (bits / 8);
        int shift = (bits - bytes_index * 8);
        *output = header[bytes_index] & (0x01 << (7 - shift));
    } while (0);
    return ret;
}

OutputEncoder::OutputEncoder() {}

OutputEncoder::~OutputEncoder() {}

int OutputEncoder::loadConfig(const char *path) {
    int ret = 0;
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
                    // // std::cout << ch_attr->Name() << " " <<
                    // ch_attr->Value() << std::endl;
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
                if (name == "ForkDisplacementSencer" ||
                    name == "Accelerometer" ||
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

int OutputEncoder::updateValue(const char *key, int len, ...) {
    int ret = 0;
    std::vector<double> calculated_input;
    std::vector<double> calculated_output;
    va_list arg_ptr;
    do {
        if (sd_ptr == nullptr) {
            ret = -1;
            break;
        }
        calculated_input.resize(len);
        va_start(arg_ptr, len);
        for (int i = 0; i < len; ++i) {
            calculated_input[i] = va_arg(arg_ptr, double);
        }
        va_end(arg_ptr);

        int handle = sd_ptr->get(key);
        std::shared_ptr<class SensorModel> ptr = sd_ptr->get(handle);
        if (ptr->isNeedParsed()) {
            ptr->solveValue(calculated_input, calculated_output);
        } else {
            ret = -2;
        }
    } while (0);
    return ret;
}

int OutputEncoder::updateSwitchValue(const char *key, int bits, bool value) {
    int ret = 0;
    struct UpdateValue ll_input = {.val = &value, .len = 1, .subId = bits};
    std::vector<struct UpdateValue *> l_input({&ll_input});
    do {
        if (sd_ptr == nullptr) {
            ret = -1;
            break;
        }
        int handle = sd_ptr->get(key);
        std::shared_ptr<class SensorModel> ptr = sd_ptr->get(handle);
        ptr->setValue(l_input);
    } while (0);
    return ret;
}
int OutputEncoder::updateValue2(const char *key, const void *input, int size) {
    int ret = 0;
    struct UpdateValue ll_input = {.val = input, .len = size, .subId = 0};
    std::vector<struct UpdateValue *> l_input({&ll_input});
    do {
        if (sd_ptr == nullptr) {
            ret = -1;
            break;
        }
        int handle = sd_ptr->get(key);
        std::shared_ptr<class SensorModel> ptr = sd_ptr->get(handle);
        ptr->setValue(l_input);
    } while (0);
    return ret;
}

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