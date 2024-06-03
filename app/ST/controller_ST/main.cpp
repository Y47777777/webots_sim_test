#include <QApplication>
#include "keyboardform/keyboardform.h"
#include "controller_ST.h"
#include "logvn/logvn.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // init glog
    // TODO:FIXME path....
    // g_Logger.initLog("../../plugins/log_config/general_controller.ini");
    // LOG_INFO("log init...");
    QApplication a(argc, argv);
    std::cout << "init" << std::endl;
    // init ctrl
    std::shared_ptr<BaseController> ctrl_ptr =
        std::make_shared<NormalSTController>();
    std::cout << "start" << std::endl;
    // ctrl_ptr->init() ;
    ctrl_ptr->start();
    std::cout << "svc_ST start" << std::endl;
    // TODO: start svc_model_ST and close svc_model_ST....
    system("./svc_ST &");
    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));

    // init keyboard
    KeyboardForm f(ctrl_ptr);
    f.show();

    a.exec();
    system("killall svc_ST");
    return 0;
}