#ifndef KEYBOARDFORM_H
#define KEYBOARDFORM_H

#include <QObject>
#include <QtWidgets/QtWidgets>
#include "controllerthrd.h"
#include <QTimer>
namespace Ui {
class KeyboardForm;
}

class KeyboardForm : public QWidget
{
    Q_OBJECT

public:
    explicit KeyboardForm(ControllerThrd* controller,QWidget *parent = nullptr);
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
    Ui::KeyboardForm *ui;

    ControllerThrd* _controller;
    double _steerSpeed;
    double _steerYaw;
    double _forkSpeed;

    QTimer _timer;
};

#endif // KEYBOARDFORM_H
