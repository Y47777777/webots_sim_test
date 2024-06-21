#include <QApplication>
#include "keyboardform/keyboardform.h"
#include "controller_P.h"
#include "logvn/logvn.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // init glog
    // TODO:FIXME path....
    g_Logger.initLog("../../plugins/log_config/general_controller.ini");
    LOG_INFO("log init...");
    QApplication a(argc, argv);
    // init ctrl
    std::shared_ptr<BaseController> ctrl_ptr =
        std::make_shared<NormalSTController>();
    // ctrl_ptr->init() ;
    ctrl_ptr->start();
    LOG_INFO("try start svc_ST");
    // TODO: start svc_model_ST and close svc_model_ST....
    system("./svc_P &");
    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));

    // init keyboard
    KeyboardForm f(ctrl_ptr);
    f.show();

    a.exec();
    LOG_INFO("try stop svc_P");
    system("killall svc_P");
    return 0;
}