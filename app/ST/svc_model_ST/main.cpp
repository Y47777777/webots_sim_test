#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "logvn/logvn.h"
#include "svc_model_serial.h"
#include "svc_model_slam_lidar.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // serial service
    std::shared_ptr<BaseSVCModel> serial_service =
        std::make_shared<SVCModelSerial>();
    serial_service->init(true, "svc_ST");
    std::shared_ptr<BaseSVCModel> lidar_service =
        std::make_shared<SVCModelLidarSlam>();
    lidar_service->init(false, "lidar");
    dynamic_cast<SVCModelLidarSlam *>(lidar_service.get())
        ->addSerialPtr(serial_service);
    bool flag = true;
    // // TODO: How to exit
    while (flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    serial_service->svcExit();
    return 0;
}