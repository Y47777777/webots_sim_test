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
    sendMsg();
}

void ConvoyerKeyboard::sendMsg() {
    if (model_ptr_ == nullptr) {
        return;
    }
    std::map<std::string, std::string> msg;
    msg["belt"] = manual_state_.selected;
    model_ptr_->onConveyorKeyboardMsg(msg);
}
