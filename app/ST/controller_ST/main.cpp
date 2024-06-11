#include <QApplication>
#include "keyboardform/keyboardform.h"
#include "controller_ST.h"
#include "logvn/logvn.h"

using namespace VNSim;

int main(int argc, char *argv[]) {
    // init glog
    // TODO:FIXME path....
    g_Logger.initLog("../../plugins/log_config/general_controller.ini");
    LOG_INFO("log init...");
    QApplication a(argc, argv);
    // init ctrl
    LOG_INFO("1");
    std::shared_ptr<BaseController> ctrl_ptr =
        std::make_shared<NormalSTController>();
    // ctrl_ptr->init() ;
    LOG_INFO("2");
    ctrl_ptr->start();
    LOG_INFO("3");
    LOG_INFO("try start svc_ST");
    std::cout << "try svc_ST" << std::endl;
    // TODO: start svc_model_ST and close svc_model_ST....
    // system("./svc_ST &");
    // std::cout << "try svc_ST finish" << std::endl;
    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));
    LOG_INFO("4");

    // init keyboard
    KeyboardForm f(ctrl_ptr);
    LOG_INFO("5");
    f.show();
    LOG_INFO("6");

    a.exec();
    LOG_INFO("try stop svc_ST");
    system("killall svc_ST");
    return 0;
}