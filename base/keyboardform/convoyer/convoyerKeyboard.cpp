#include "ui_convoyerKeyboard.h"
#include "convoyerKeyboard.h"
#include "logvn/logvn.h"

using namespace VNSim;

ConvoyerKeyboard::ConvoyerKeyboard(std::shared_ptr<BaseController> ptr, QWidget *parent)
    : QWidget(parent), ui(new Ui::ConvoyerKeyboard) {
    ui->setupUi(this);
    model_ptr_ = ptr;
    //timer_.setInterval(50);
    //connect(&timer_, SIGNAL(timeout()), this, SLOT(setStatus()));
    //timer_.start();
    if (model_ptr_ == nullptr) {
        LOG_ERROR("model_ptr_ is nullptr");
    }
}

ConvoyerKeyboard::~ConvoyerKeyboard() {
    delete ui;
}

void ConvoyerKeyboard::on_pushButton_clicked(){
    manual_state_.selected = ui->TargetConveyorText->text().toStdString();
    manual_state_.function = "0";
    sendMsg();
}

void ConvoyerKeyboard::on_pushButton_remove_clicked(){
    manual_state_.selected = ui->TargetConveyorText->text().toStdString();
    manual_state_.function = "1";
    sendMsg();
}

// void ConvoyerKeyboard::on_checkBox_stateChanged(int arg1){
//     manual_state_.manual = ui->checkBox->isChecked() ? "true" : "false";
//     manual_state_.function = "1";
//     sendMsg();
// }

void ConvoyerKeyboard::sendMsg() {
    if (model_ptr_ == nullptr) {
        return;
    }
    std::map<std::string, std::string> msg;
    msg["belt"] = manual_state_.selected;
    msg["function"] = manual_state_.function;
    model_ptr_->onConveyorKeyboardMsg(msg);
}
