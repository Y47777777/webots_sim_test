/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-07 15:57:36
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 17:31:55
 * @FilePath: /webots_ctrl/include/data_process_service/sensor_process.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once
#include <thread>
#include <shared_mutex>
#include <stdint.h>
#include <memory>

#include "Parser/parser.hpp"
#include "logvn/logvn.h"
#include "Parser/parser.hpp"
#include "time/time.h"
#include "ecal_wrapper/ecal_wrapper.h"

namespace VNSim {

class SensorProcessBase {};

}  // namespace VNSim