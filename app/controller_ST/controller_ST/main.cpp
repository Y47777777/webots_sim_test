#include <QApplication>
#include "base_ctrl.h"
#include "keyboardform/keyboardform.h"
#include "controller_ST.h"

int main(int argc, char *argv[]) {
    // g_Logger.initLog("/home/vision/webots/generalsim/plugins/log_config/general_controller.ini");
    // LOG_INFO("log init...");
    QApplication a(argc, argv);
    std::shared_ptr<BaseController> ctrl_ptr =
        std::make_shared<NormalSTController>();
    ctrl_ptr->init();
    ctrl_ptr->start();
    // TODO: start svc_model_ST....
    QObject::connect(ctrl_ptr.get(), SIGNAL(finished()), &a, SLOT(quit()));
    KeyboardForm f(ctrl_ptr);
    f.show();
    return a.exec();
}