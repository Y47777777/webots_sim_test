#ifndef __KEYBORADFORM_H__
#define __KEYBORADFORM_H__

#include <QObject>
#include <QTimer>
#include <QtWidgets/QtWidgets>
#include <map>
#include "base_ctrl.h"

namespace Ui {
class KeyboardForm;
}

class KeyboardForm : public QWidget {
    Q_OBJECT
   private:
    Ui::KeyboardForm *ui;

    std::shared_ptr<BaseController> model_ptr_;
    std::map<std::string, double> input_msg_;
    std::map<std::string, double> output_msg_;

    QTimer timer_;

   public:
    explicit KeyboardForm(std::shared_ptr<BaseController> ptr,
                          QWidget *parent = nullptr);
    // explicit KeyboardForm(QWidget *parent = nullptr);
    ~KeyboardForm();

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
    void setStatus();

   private:
    void sendMsg();

   public:
    static void decodeMsg(const char *msg,
                          std::map<std::string, double> &decode_msg);
    static void encodeMsg(const std::map<std::string, double> &msg,
                          std::string &encode_msg);
};

#endif  // KEYBOARDFORM_H
