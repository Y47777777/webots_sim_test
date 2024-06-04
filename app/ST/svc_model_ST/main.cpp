#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "logvn/logvn.h"
#include "svc_model_serial.h"
#include "svc_model_slam_lidar.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // serial service
    g_Logger.initLog("../../plugins/log_config/general_controller.ini");
    LOG_INFO("start svc_ST");
    std::shared_ptr<BaseSVCModel> serial_service =
        std::make_shared<SVCModelSerial>();
    serial_service->init(true, "svc_ST");
    std::shared_ptr<BaseSVCModel> lidar_service =
        std::make_shared<SVCModelLidarSlam>();
    lidar_service->init(false, "lidar");
    bool flag = true;
    // TODO: How to exit
    while (flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    serial_service->svcExit();
    return 0;
}