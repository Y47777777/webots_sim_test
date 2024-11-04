#include <QApplication>
#include <unistd.h>
#include "keyboardform/keyboardform.h"
#include "keyboardform/convoyerKeyboard.h"
#include "R_master.h"
#include "logvn/logvn.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // init glog
    // TODO:FIXME path....
    g_Logger.initLog("../../configs/log_config/webots_log_config.ini","/home/visionnav/logs/master.log");
    
    LOG_INFO("log init...");
    LOG_INFO("try start svc_R14");
    system("./../svc_R14/svc_R14 &");
    QApplication a(argc, argv);
    // init ctrl
    std::shared_ptr<BaseController> ctrl_ptr = std::make_shared<AGVController>();

    ctrl_ptr->start();

    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));

    // init keyboard
    KeyboardForm f(ctrl_ptr);
    f.show();
    
    ConvoyerKeyboard f2(ctrl_ptr);
    f2.show();

    a.exec();
    LOG_INFO("try stop svc_R14");
    system("killall svc_R14");
    
    return 0;
}