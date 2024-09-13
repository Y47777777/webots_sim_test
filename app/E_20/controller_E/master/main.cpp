#include <QApplication>
#include "keyboardform/keyboardform.h"
#include "keyboardform/convoyerKeyboard.h"
#include "E_master.h"
#include "logvn/logvn.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // init glog
    // TODO:FIXME path....
    g_Logger.initLog("../../plugins/log_config/webots_master.ini");

    LOG_INFO("log init...");
    LOG_INFO("try start svc_E20");
    system("./../svc_E_20/svc_E_20 &");

    QApplication a(argc, argv);
    // init ctrl
    std::shared_ptr<BaseController> ctrl_ptr =
        std::make_shared<AGVController>();

    ctrl_ptr->start();

    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));

    // init keyboard
    KeyboardForm f(ctrl_ptr);
    f.show();

    ConvoyerKeyboard f2(ctrl_ptr);
    f2.show();

    a.exec();
    LOG_INFO("try stop svc_E20");
    system("killall svc_E_20");

    return 0;
}