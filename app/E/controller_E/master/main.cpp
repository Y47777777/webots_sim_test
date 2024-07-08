#include <QApplication>
#include "keyboardform/keyboardform.h"
#include "E_master.h"
#include "logvn/logvn.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // init glog
    // TODO:FIXME path....
    g_Logger.initLog("../../plugins/log_config/webots_master.ini");

    LOG_INFO("log init...");
    QApplication a(argc, argv);
    // init ctrl
    printf("1\n");
    std::shared_ptr<BaseController> ctrl_ptr =
        std::make_shared<AGVController>();
    printf("2\n");
    ctrl_ptr->start();
    printf("3\n");

    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));

    // init keyboard
    KeyboardForm f(ctrl_ptr);
    f.show();

    a.exec();
    // LOG_INFO("try stop svc_P");
    // system("killall svc_E");

    return 0;
}