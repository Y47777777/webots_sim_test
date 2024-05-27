#include "keyboardform.h"
#include "ui_keyboardform.h"

KeyboardForm::KeyboardForm(ControllerThrd* controller,QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::KeyboardForm)
{
    ui->setupUi(this);
    _controller=controller;
    _steerSpeed=0;
    _steerYaw=0;
    _forkSpeed=0;

    _timer.setInterval(50);
    connect(&_timer,SIGNAL(timeout()),this,SLOT(setStatus()));
    _timer.start();
}

KeyboardForm::~KeyboardForm()
{
    delete ui;
}

void KeyboardForm::setStatus()
{
    double steerSpeed;
    double steerYaw;
    double forkSpeed;
    double forkHeight;
    _controller->getStatus(steerSpeed,steerYaw,forkSpeed,forkHeight);
    ui->label_steerSpeed_value->setText(QString::number(steerSpeed));
    ui->label_steerYaw_value->setText(QString::number(steerYaw*180/3.14));
    ui->label_forkSpeed_value->setText(QString::number(forkSpeed));
    ui->label_forkHeight_value->setText(QString::number(forkHeight));
}

void KeyboardForm::on_checkBox_stateChanged(int arg1)
{
    _controller->setMenual(ui->checkBox->isChecked());
    _steerSpeed=0;
    _steerYaw=0;
    _forkSpeed=0;
}


void KeyboardForm::on_pushButton_speedUp_clicked()
{
    if(_steerSpeed<2)
        _steerSpeed+=0.5;
    _controller->speedChanged(_steerSpeed,_steerYaw,_forkSpeed);
}


void KeyboardForm::on_pushButton_stop_clicked()
{
    _steerSpeed=0;
    _controller->speedChanged(_steerSpeed,_steerYaw,_forkSpeed);
}


void KeyboardForm::on_pushButton_speedDown_clicked()
{
    if(_steerSpeed>-2)
        _steerSpeed-=0.5;
    _controller->speedChanged(_steerSpeed,_steerYaw,_forkSpeed);
}


void KeyboardForm::on_pushButton_turnLeft_clicked()
{
    if(_steerYaw<1.57)
        _steerYaw+=0.26;
    _controller->speedChanged(_steerSpeed,_steerYaw,_forkSpeed);
}

void KeyboardForm::on_pushButton_turnRight_clicked()
{
    if(_steerYaw>-1.57)
        _steerYaw-=0.26;
    _controller->speedChanged(_steerSpeed,_steerYaw,_forkSpeed);
}

void KeyboardForm::on_pushButton_liftUp_clicked()
{
    if(_forkSpeed<1)
        _forkSpeed+=0.1;
    _controller->speedChanged(_steerSpeed,_steerYaw,_forkSpeed);
}


void KeyboardForm::on_pushButton_liftDown_clicked()
{
    if(_forkSpeed>-1)
        _forkSpeed-=0.1;
    _controller->speedChanged(_steerSpeed,_steerYaw,_forkSpeed);
}


void KeyboardForm::on_pushButton_liftStop_clicked()
{
    _forkSpeed=0;
    _controller->speedChanged(_steerSpeed,_steerYaw,_forkSpeed);
}





























