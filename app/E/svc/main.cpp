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

#include "svc_ctrl.h"
#include "svc_sensor.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // serial service
    // TODO: 生成不同的log
    g_Logger.initLog("../../configs/log_config/general_svc.ini");
    LOG_INFO("start svc_E");

    std::shared_ptr<BaseSvc> serial_service = std::make_shared<SVCMaster>();
    serial_service->init(true, "svc_E");

    std::shared_ptr<BaseSvc> lidar_service = std::make_shared<SVCShadow>();
    lidar_service->init(false, "lidar");

    bool flag = true;
    // TODO: How to exit
    while (flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    serial_service->svcExit();
    return 0;
}