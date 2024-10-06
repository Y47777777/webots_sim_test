#include <QApplication>
#include "keyboardform/keyboardform.h"
#include "E_master.h"
#include "logvn/logvn.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // init glog
    // TODO:FIXME path....
    g_Logger.initLog("../../configs/log_config/webots_master.ini");

    LOG_INFO("log init...");
    LOG_INFO("try start svc_E_four_forks");
    system("./../svc_E_four_forks/svc_E_four_forks &");

    QApplication a(argc, argv);
    // init ctrl
    std::shared_ptr<BaseController> ctrl_ptr =
        std::make_shared<AGVController>();

    ctrl_ptr->start();

    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));

    // init keyboard
    KeyboardForm f(ctrl_ptr);
    f.show();

    a.exec();
    LOG_INFO("try stop svc_E_four_forks");
    system("killall svc_E_four_forks");

    return 0;
}