#include <QApplication>
#include "keyboardform/keyboardform.h"
#include "shadow_be.h"
#include "logvn/logvn.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // init glog
    // TODO:FIXME path....
    g_Logger.initLog("../../configs/log_config/webots_log_config.ini","/home/visionnav/logs/shadow_be.log");
    LOG_INFO("log init...");
    QApplication a(argc, argv);
    // init ctrl
    std::shared_ptr<BaseController> ctrl_ptr = std::make_shared<AGVController>();

    ctrl_ptr->start();

    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));

    a.exec();
    // LOG_INFO("try stop svc_P");
    // system("killall svc_P");
    return 0;
}