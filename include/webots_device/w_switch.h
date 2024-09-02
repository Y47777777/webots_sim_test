#ifndef __W_SWITCH_H__
#define __W_SWITCH_H__
#include "w_base.h"
#include <webots/DistanceSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <Eigen/Core>
namespace VNSim {

class WSwitch : public WBase {
   public:
    WSwitch() = default;
    virtual bool getValue() = 0;

    virtual Eigen::Matrix4d getToBase() = 0;

    virtual Eigen::Matrix4d getToWorld() = 0;

    virtual ~WSwitch() {}

   protected:
    bool switchTag;
};

class manchanical : public WSwitch {
   public:
    manchanical(const std::string &device_name, const std::string &robot_def, int freq = 100) {
        device = super_->getTouchSensor(device_name);
        if (device){
            deviceNode = super_->getFromDevice(device);
            device->enable(freq);
        }
        else
            deviceNode = nullptr;
        robotNode = super_->getFromDef(robot_def);

        deviceName = device_name;
        robotName = robot_def;
        switchTag = false;
    }

    void spin() override {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        if (device) {
            auto curr_val = device->getValue();
            switchTag = (curr_val == 1.0 ? true : false);
            LOG_DEBUG("registed device = %s --> %d, value = %lf", deviceName.c_str(), switchTag, curr_val);
        } else {
            std::cerr << " registed device :" << deviceName << " is null"
                      << std::endl;
            switchTag = false;
        }
    }

    bool getValue() override {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return switchTag;
    }

    /**
     * @brief Get the To Base object
     *
     * @return Eigen::Matrix4d
     * @throw  if the inputed robot_def or device not found
     */
    Eigen::Matrix4d getToBase() override {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        if (deviceNode == nullptr) {
            char error_str[256];
            sprintf(error_str, "%s @error : device \'%s\' not found",
                    __FUNCTION__, deviceName.c_str());
            throw std::runtime_error(error_str);
        }
        if (robotNode == nullptr) {
            char error_str[256];
            if (robotName.empty()) {
                sprintf(error_str, "%s @error : inputed robot name is empty",
                        __FUNCTION__);
                throw std::runtime_error(error_str);
            } else {
                sprintf(error_str, "%s @error : robot def \'%s\' not found",
                        __FUNCTION__, robotName.c_str());
                throw std::runtime_error(error_str);
            }
        }
        auto pose_array = deviceNode->getPose(robotNode);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) { ret(i, j) = pose_array[i * 4 + j]; }
        }
        return ret;
    }

    /**
     * @brief Get the To World object
     *
     * @return Eigen::Matrix4d
     * @throw  if the inputed device not found
     */
    Eigen::Matrix4d getToWorld() override {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        if (deviceNode == nullptr) {
            char error_str[256];
            sprintf(error_str, "%s @error : device \'%s\' not found",
                    __FUNCTION__, deviceName.c_str());
            throw std::runtime_error(error_str);
        }
        auto pose_array = deviceNode->getPose();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) { ret(i, j) = pose_array[i * 4 + j]; }
        }
        return ret;
    }

   protected:
    webots::TouchSensor *device;
    webots::Node *deviceNode;
    webots::Node *robotNode;
    std::string robotName;
    std::string deviceName;
};

class photoelectric : public WSwitch {
   public:
    photoelectric(const std::string &device_name,
                  const std::string &robot_def, int freq = 100) {
        device = super_->getDistanceSensor(device_name);
        robotNode = super_->getFromDef(robot_def);
        if (device){
            deviceNode = super_->getFromDevice(device);
            device->enable(freq);
        }
        else
            deviceNode = nullptr;
        if (deviceNode){
            deviceField = deviceNode->getField("signThreshold");
        }
        else{
            deviceField = nullptr;
        }
        deviceName = device_name;
        robotName = robot_def;
        switchTag = false;
    }

    void spin() override {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        if (device) {
            if (deviceNode == nullptr) {
                deviceNode = super_->getFromDevice(device);
                deviceField = deviceNode->getField("signThreshold");
            }
            auto thresholdHigh = deviceField->getSFFloat();
            auto curr_val = device->getValue();

            switchTag = ((curr_val <= thresholdHigh) ? true : false);
            LOG_DEBUG("registed device = %s --> %d, target = %lf, value = %lf", deviceName.c_str(), switchTag, thresholdHigh, curr_val);
        } else { 
            std::cerr << " registed device :" << deviceName << " is null"
                      << std::endl;
            switchTag = false;
        }
    }

    bool getValue() override {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return switchTag;
    }

    /**
     * @brief Get the To Base object
     *
     * @return Eigen::Matrix4d
     * @throw  if the inputed robot_def or device not found
     */
    Eigen::Matrix4d getToBase() override {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        if (deviceNode == nullptr) {
            char error_str[256];
            sprintf(error_str, "%s @error : device \'%s\' not found",
                    __FUNCTION__, deviceName.c_str());
            throw std::runtime_error(error_str);
        }
        if (robotNode == nullptr) {
            char error_str[256];
            if (robotName.empty()) {
                sprintf(error_str, "%s @error : inputed robot name is empty",
                        __FUNCTION__);
                throw std::runtime_error(error_str);
            } else {
                sprintf(error_str, "%s @error : robot def \'%s\' not found",
                        __FUNCTION__, robotName.c_str());
                throw std::runtime_error(error_str);
            }
        }
        auto pose_array = deviceNode->getPose(robotNode);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) { ret(i, j) = pose_array[i * 4 + j]; }
        }
        return ret;
    }

    /**
     * @brief Get the To World object
     *
     * @return Eigen::Matrix4d
     * @throw  if the inputed device not found
     */
    Eigen::Matrix4d getToWorld() override {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
        if (deviceNode == nullptr) {
            char error_str[256];
            sprintf(error_str, "%s @error : device \'%s\' not found",
                    __FUNCTION__, deviceName.c_str());
            throw std::runtime_error(error_str);
        }
        auto pose_array = deviceNode->getPose();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) { ret(i, j) = pose_array[i * 4 + j]; }
        }
        return ret;
    }

   protected:
    webots::DistanceSensor *device;
    webots::Node *deviceNode;
    webots::Node *robotNode;
    webots::Field *deviceField;
    std::string deviceName;
    std::string robotName;
};

}  // namespace VNSim
#endif  // !__W_SWITCH_H__