#include <QApplication>
#include <unistd.h>
#include "keyboardform/keyboardform.h"
#include "P_master.h"
#include "logvn/logvn.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // init glog
    // TODO:FIXME path....
    g_Logger.initLog("../../configs/log_config/webots_log_config.ini","/home/visionnav/logs/master.log");
    
    LOG_INFO("log init...");
    LOG_INFO("try start svc_P15");
    system("./../svc_P15/svc_P15 &");
    QApplication a(argc, argv);
    // init ctrl
    std::shared_ptr<BaseController> ctrl_ptr = std::make_shared<AGVController>();

    ctrl_ptr->start();

    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));

    // init keyboard
    KeyboardForm f(ctrl_ptr);
    f.show();

    a.exec();
    LOG_INFO("try stop svc_P15");
    system("killall svc_P15");
    
    return 0;
}