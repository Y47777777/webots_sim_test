#include "ui_keyboardform.h"
#include "keyboardform.h"
#include "logvn/logvn.h"

using namespace VNSim;

KeyboardForm::KeyboardForm(std::shared_ptr<BaseController> ptr, QWidget *parent)
    : QWidget(parent), ui(new Ui::KeyboardForm) {
    ui->setupUi(this);
    model_ptr_ = ptr;
    timer_.setInterval(300);
    connect(&timer_, SIGNAL(timeout()), this, SLOT(setStatus()));
    timer_.start();
    if (model_ptr_ == nullptr) {
        LOG_ERROR("model_ptr_ is nullptr");
    }
}

KeyboardForm::~KeyboardForm() {
    delete ui;
}

void KeyboardForm::setStatus() {
    if (model_ptr_ == nullptr) {
        return;
    }
    std::map<std::string, double> msg;
    model_ptr_->manualGetState(msg);

    if (msg.find("steer_speed") != msg.end()) {
        ui->label_steerSpeed_value->setText(QString::number(msg["steer_speed"]));
    }

    if (msg.find("steer_yaw") != msg.end()) {
        ui->label_steerYaw_value->setText(QString::number(msg["steer_yaw"] * 180 / 3.14));
    }

    if (msg.find("fork_speed") != msg.end()) {
        ui->label_forkSpeed_value->setText(QString::number(msg["fork_speed"]));
    }

    if (msg.find("fork_height") != msg.end()) {
        ui->label_forkHeight_value->setText(QString::number(msg["fork_height"]));
    }

    if (msg.find("forkX_speed") != msg.end()) {
        ui->label_forkXSpeed_value->setText(QString::number(msg["forkX_speed"]));
    }

    if (msg.find("forkX_height") != msg.end()) {
        ui->label_forkXHeight_value->setText(QString::number(msg["forkX_height"]));
    }

    if (msg.find("forkY_speed") != msg.end()) {
        ui->label_forkYSpeed_value->setText(QString::number(msg["forkY_speed"]));
    }

    if (msg.find("forkY_height") != msg.end()) {
        ui->label_forkYHeight_value->setText(QString::number(msg["forkY_height"]));
    }

    if (msg.find("forkP_speed") != msg.end()) {
        ui->label_forkPSpeed_value->setText(QString::number(msg["forkP_speed"]));
    }

    if (msg.find("forkP_height") != msg.end()) {
        ui->label_forkPHeight_value->setText(QString::number(msg["forkP_height"]));
    }

    if (msg.find("forkC_speed") != msg.end()) {
        ui->label_forkCSpeed_value->setText(QString::number(msg["forkC_speed"]));
    }

    if (msg.find("forkC_height") != msg.end()) {
        ui->label_forkCHeight_value->setText(QString::number(msg["forkC_height"]));
    }

    if (msg.find("forkCR_height") != msg.end()) {
        ui->label_forkCRHeight_value->setText(QString::number(msg["forkCR_height"]));
    }

    if (msg.find("forkCL_height") != msg.end()) {
        ui->label_forkCLHeight_value->setText(QString::number(msg["forkCL_height"]));
    }

    if (msg.find("forkC_force") != msg.end()) {
        ui->label_forkCForce_value->setText(QString::number(msg["forkC_force"]));
    }

    if (msg.find("real_speed") != msg.end()) {
        ui->label_realspeed_value->setText(QString::number(msg["real_speed"]));
    }

    if (msg.find("lidar0_Height") != msg.end()) {
        ui->label_Lidar0Height_value->setText(QString::number(msg["lidar0_Height"]));
    }

    if (msg.find("lidar0_isInitHeight") != msg.end()) {
        ui->label_isLidar0ORHeight_value->setText(msg["lidar0_isInitHeight"] == 1 ? "Lidia0在初始高度"
                                                                                  : "Lidia0不在初始高度");
    }
}

void KeyboardForm::on_checkBox_stateChanged(int arg1) {
    if (model_ptr_ == nullptr) {
        return;
    }
    model_ptr_->shiftControlMode(ui->checkBox->isChecked());
    manual_state_.steer_speed = 0;
    manual_state_.steer_yaw = 0;
    manual_state_.fork_speed = 0;
    manual_state_.forkX_speed = 0;
    manual_state_.forkY_speed = 0;
    manual_state_.forkP_speed = 0;
    manual_state_.forkC_speed = 0;
    manual_state_.real_speed = 0;
    manual_state_.lidar0_move = 0;
    manual_state_.lidar0_height = 0;
}

void KeyboardForm::on_pushButton_speedUp_clicked() {
    double c_value = manual_state_.steer_speed;
    if (c_value < 4) {
        ;
        c_value += 0.2;
    }
    manual_state_.steer_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_stop_clicked() {
    manual_state_.steer_speed = 0;
    sendMsg();
}

void KeyboardForm::on_pushButton_speedDown_clicked() {
    double c_value = manual_state_.steer_speed;
    if (c_value > -4) {
        c_value -= 0.2;
    }
    manual_state_.steer_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_turnLeft_clicked() {
    double c_value = manual_state_.steer_yaw;
    if (c_value < 1.57) {
        c_value += 0.16;
    }
    if (std::abs(c_value) > 1.57) {
        c_value = 1.57;
    }
    manual_state_.steer_yaw = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_turnRight_clicked() {
    double c_value = manual_state_.steer_yaw;
    if (c_value > -1.57) {
        c_value -= 0.16;
    }
    if (std::abs(c_value) > 1.57) {
        c_value = -1.57;
    }
    manual_state_.steer_yaw = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftUp_clicked() {
    double c_value = manual_state_.fork_speed;
    if (c_value < 1) {
        c_value += 0.1;
    }
    manual_state_.fork_speed = c_value;
    sendMsg();
}
void KeyboardForm::on_pushButton_liftStop_clicked() {
    manual_state_.fork_speed = 0;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftDown_clicked() {
    double c_value = manual_state_.fork_speed;
    if (c_value > -1) {
        c_value -= 0.1;
    }
    manual_state_.fork_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftXForward_clicked() {
    double c_value = manual_state_.forkX_speed;
    if (c_value < 1) {
        c_value += 0.1;
    }
    manual_state_.forkX_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftXStop_clicked() {
    manual_state_.forkX_speed = 0;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftXBackward_clicked() {
    double c_value = manual_state_.forkX_speed;
    if (c_value > -1) {
        c_value -= 0.1;
    }
    manual_state_.forkX_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftYLeft_clicked() {
    double c_value = manual_state_.forkY_speed;
    if (c_value > -1) {
        c_value -= 0.1;
    }
    manual_state_.forkY_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftYRight_clicked() {
    double c_value = manual_state_.forkY_speed;
    if (c_value < 1) {
        c_value += 0.1;
    }
    manual_state_.forkY_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftYStop_clicked() {
    manual_state_.forkY_speed = 0;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftPDown_clicked() {
    double c_value = manual_state_.forkP_speed;
    if (c_value > -1) {
        c_value -= 0.01;
    }
    manual_state_.forkP_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftPUp_clicked() {
    double c_value = manual_state_.forkP_speed;
    if (c_value < 1) {
        c_value += 0.01;
    }
    manual_state_.forkP_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftPStop_clicked() {
    manual_state_.forkP_speed = 0;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftCOpen_clicked() {
    double c_value = manual_state_.forkC_speed;
    if (c_value < 1) {
        c_value += 0.01;
    }
    manual_state_.forkC_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftCClose_clicked() {
    double c_value = manual_state_.forkC_speed;
    if (c_value > -1) {
        c_value -= 0.01;
    }
    manual_state_.forkC_speed = c_value;
    sendMsg();
}

void KeyboardForm::on_pushButton_liftCStop_clicked() {
    manual_state_.forkC_speed = 0;
    sendMsg();
}

void KeyboardForm::on_pushButton_refresh_clicked() {
    manual_state_.refresh_world = true;
    sendMsg();
}

void KeyboardForm::on_pushButton_reset_steer_clicked() {
    manual_state_.steer_yaw = 0;
    sendMsg();
}

void KeyboardForm::on_pushButton_lidar0Up_clicked() {
    manual_state_.lidar0_move = 1;
    sendMsg();
}

void KeyboardForm::on_pushButton_lidar0Stop_clicked() {
    manual_state_.lidar0_move = 0;
    sendMsg();
}

void KeyboardForm::on_pushButton_Lidar0Down_clicked() {
    manual_state_.lidar0_move = -1;
    sendMsg();
}

void KeyboardForm::sendMsg() {
    if (model_ptr_ == nullptr) {
        return;
    }
    std::map<std::string, double> msg;
    msg["steer_speed"] = manual_state_.steer_speed;
    msg["steer_yaw"] = manual_state_.steer_yaw;
    msg["fork_speed"] = manual_state_.fork_speed;
    msg["fork_height"] = manual_state_.fork_height;
    msg["forkX_speed"] = manual_state_.forkX_speed;
    msg["forkX_height"] = manual_state_.forkX_height;
    msg["forkY_speed"] = manual_state_.forkY_speed;
    msg["forkY_height"] = manual_state_.forkY_height;
    msg["forkP_speed"] = manual_state_.forkP_speed;
    msg["forkP_height"] = manual_state_.forkP_height;
    msg["forkC_speed"] = manual_state_.forkC_speed;
    msg["forkC_height"] = manual_state_.forkC_height;
    msg["real_speed"] = manual_state_.real_speed;
    msg["lidar0_move"] = manual_state_.lidar0_move;
    msg["lidar0_height"] = manual_state_.lidar0_height;

    if (manual_state_.refresh_world) {
        msg["refresh_world"] = 0;
        manual_state_.refresh_world = false;
    }
    model_ptr_->manualSetState(msg);
}

void KeyboardForm::decodeMsg(const char *msg, std::map<std::string, double> &decode_msg) {}
void KeyboardForm::encodeMsg(const std::map<std::string, double> &msg, std::string &encode_msg) {}
