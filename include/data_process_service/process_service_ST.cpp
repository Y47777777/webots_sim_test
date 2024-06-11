/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-07 16:27:22
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 17:23:37
 * @FilePath: /webots_ctrl/app/ST/svc_model_ST/process_service_ST.cpp
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#include "process_service_ST.h"
using namespace VNSim;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;

STProcessService::STProcessService() {}
STProcessService::~STProcessService() {}

void STProcessService::fromCtrlSystem(uint8_t *msg, int len) {}

void STProcessService::toCtrlSystem() {}

void STProcessService::fromSimSystem(uint8_t *msg, int len) {}

void STProcessService::toSimSystem() {}