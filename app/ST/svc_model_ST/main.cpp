/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 12:21:35
 * @FilePath: /webots_ctrl/app/ST/svc_model_ST/main.cpp
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "logvn/logvn.h"
#include "svc_model_serial.h"
#include "svc_model_slam_lidar.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // serial service
    // TODO: 生成不同的log
    g_Logger.initLog("../../plugins/log_config/general_controller.ini");
    LOG_INFO("start svc_ST");
    std::shared_ptr<BaseSVCModel> serial_service =
        std::make_shared<SVCModelSerial>();
    serial_service->init(true, "svc_ST");
    std::shared_ptr<BaseSVCModel> lidar_service =
        std::make_shared<SVCModelLidar>();
    lidar_service->init(false, "lidar");
    bool flag = true;
    // TODO: How to exit
    while (flag) {
        std::cout << "svc_running..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    serial_service->svcExit();
    return 0;
}