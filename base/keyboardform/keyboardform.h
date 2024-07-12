#ifndef __KEYBORADFORM_H__
#define __KEYBORADFORM_H__

#include <QObject>
#include <QTimer>
#include <QtWidgets/QtWidgets>
#include <map>
#include "controller/base_ctrl.h"

namespace Ui {
class KeyboardForm;
}

class KeyboardForm : public QWidget {
    Q_OBJECT

   public:
    explicit KeyboardForm(std::shared_ptr<VNSim::BaseController> ptr,
                          QWidget *parent = nullptr);

    ~KeyboardForm();

    static void decodeMsg(const char *msg,
                          std::map<std::string, double> &decode_msg);
    static void encodeMsg(const std::map<std::string, double> &msg,
                          std::string &encode_msg);

   private slots:
    void on_checkBox_stateChanged(int arg1);
    void on_pushButton_speedUp_clicked();
    void on_pushButton_stop_clicked();
    void on_pushButton_speedDown_clicked();
    void on_pushButton_turnLeft_clicked();
    void on_pushButton_turnRight_clicked();
    void on_pushButton_liftUp_clicked();
    void on_pushButton_liftDown_clicked();
    void on_pushButton_liftStop_clicked();
    void on_pushButton_liftYLeft_clicked();
    void on_pushButton_liftYRight_clicked();
    void on_pushButton_liftYStop_clicked();
    void on_pushButton_liftPDown_clicked();
    void on_pushButton_liftPUp_clicked();
    void on_pushButton_liftPStop_clicked();
    void on_pushButton_liftCOpen_clicked();
    void on_pushButton_liftCClose_clicked();
    void on_pushButton_liftCStop_clicked();
    void setStatus();

   private:
    void sendMsg();
    struct ManualState {
        double steer_speed = 0;
        double steer_yaw = 0;
        double fork_speed = 0;
        double fork_height = 0;
        double forkY_speed = 0;
        double forkY_height = 0;
        double forkP_speed = 0;
        double forkP_height = 0;
        double forkC_speed = 0;
        double forkC_height = 0;
        double real_speed = 0;
    };
    ManualState manual_state_;

    Ui::KeyboardForm *ui;

    std::shared_ptr<VNSim::BaseController> model_ptr_;

    QTimer timer_;
};

#endif  // KEYBOARDFORM_H
