#include <QApplication>
#include "logvn.h"
#include "controllerthrd.h"
#include "keyboardform.h"
using namespace VNSim;
int main(int argc, char *argv[])
{
    g_Logger.initLog("/home/vision/webots/SimRobot/plugins/log_config/general_controller.ini");
    LOG_INFO("log init...");

    QApplication a(argc,argv);



    ControllerThrd controllr;
    controllr.start();

    QObject::connect(&controllr,SIGNAL(finished()),&a,SLOT(quit()));

    KeyboardForm f(&controllr);

    f.show();

    return a.exec();
}





























