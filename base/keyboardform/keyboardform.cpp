#include "ui_keyboardform.h"
#include "nlohmann/json.hpp"
#include "keyboardform/keyboardform.h"

KeyboardForm::KeyboardForm(std::shared_ptr<BaseController> ptr, QWidget *parent)
    : QWidget(parent), ui(new Ui::KeyboardForm) {
    ui->setupUi(this);
    model_ptr_ = ptr;

    timer_.setInterval(50);
    connect(&timer_, SIGNAL(timeout()), this, SLOT(setStatus()));
    timer_.start();
    // Init Keyboard
    output_msg_["steerSpeed"] = 0;
    output_msg_["steerYaw"] = 0;
    output_msg_["forkSpeed"] = 0;
}

KeyboardForm::~KeyboardForm() {
    delete ui;
}

void KeyboardForm::setStatus() {
    std::string msg;
    model_ptr_->onManualReport(msg);
    // decode msg
    decodeMsg(msg.c_str(), input_msg_);
    ui->label_steerSpeed_value->setText(
        QString::number(input_msg_["steerSpeed"]));
    ui->label_steerYaw_value->setText(
        QString::number(input_msg_["steerYaw"] * 180 / 3.14));
    ui->label_forkSpeed_value->setText(
        QString::number(input_msg_["forkSpeed"]));
    ui->label_forkHeight_value->setText(
        QString::number(input_msg_["forkHeight"]));
    ui->label_realspeed_value->setText(
        QString::number(input_msg_["realSpeed"]));
}

void KeyboardForm::on_checkBox_stateChanged(int arg1) {
    model_ptr_->shiftControlMode(ui->checkBox->isChecked());
    output_msg_["steerSpeed"] = 0;
    output_msg_["steerYaw"] = 0;
    output_msg_["forkSpeed"] = 0;
}

void KeyboardForm::on_pushButton_speedUp_clicked() {
    double c_value = output_msg_["steerSpeed"];
    if (c_value < 4) {
        c_value += 0.5;
    }
    output_msg_["steerSpeed"] = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_stop_clicked() {
    output_msg_["steerSpeed"] = 0;
    sendMsg();
}

void KeyboardForm::on_pushButton_speedDown_clicked() {
    double c_value = output_msg_["steerSpeed"];
    if (c_value > -4) {
        c_value -= 0.5;
    }
    output_msg_["steerSpeed"] = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_turnLeft_clicked() {
    double c_value = output_msg_["steerYaw"];
    if (c_value < 1.57) {
        c_value += 0.26;
    }
    if (std::abs(c_value) > 1.57) {
        c_value = 1.57;
    }
    output_msg_["steerYaw"] = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_turnRight_clicked() {
    double c_value = output_msg_["steerYaw"];
    if (c_value > -1.57) {
        c_value -= 0.26;
    }
    if (std::abs(c_value) > 1.57) {
        c_value = -1.57;
    }
    output_msg_["steerYaw"] = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftUp_clicked() {
    double c_value = output_msg_["forkSpeed"];
    if (c_value < 1) {
        c_value += 0.1;
    }
    output_msg_["forkSpeed"] = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftDown_clicked() {
    double c_value = output_msg_["forkSpeed"];
    if (c_value > -1) {
        c_value -= 0.1;
    }
    output_msg_["forkSpeed"] = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftStop_clicked() {
    output_msg_["forkSpeed"] = 0;
    sendMsg();
}

void KeyboardForm::decodeMsg(const char *msg,
                             std::map<std::string, double> &decode_msg) {
    nlohmann::json json_obj = nlohmann::json::parse(msg);
    for (auto it = json_obj.begin(); it != json_obj.end(); ++it) {
        decode_msg[it.key()] = it.value();
    }
}

void KeyboardForm::encodeMsg(const std::map<std::string, double> &msg,
                             std::string &encode_msg) {
    nlohmann::json json_obj(msg);
    encode_msg = json_obj.dump();
}

void KeyboardForm::sendMsg() {
    std::string encode_msg;
    encodeMsg(output_msg_, encode_msg);
    model_ptr_->onManualMsg(encode_msg.c_str());
}
