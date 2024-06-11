/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 16:37:03
 * @FilePath: /webots_ctrl/app/ST/svc_model_ST/main_tmp.cpp
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#include <stdlib.h>
#include <thread>
#include "logvn/logvn.h"
#include "process_service_ST.h"


using namespace VNSim;

int main(int argc, char *argv[]) {
    // serial service
    // TODO: 生成不同的log
    g_Logger.initLog("../../plugins/log_config/general_controller.ini");
    LOG_INFO("start svc_ST");

    bool flag = true;
    // TODO: How to exit
    while (flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}