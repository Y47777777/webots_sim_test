#ifndef __CONVOYERKEYBORADFORM_H__
#define __CONVOYERKEYBORADFORM_H__

#include <QObject>
#include <QTimer>
#include <QtWidgets/QtWidgets>
#include <map>
#include "controller/base_ctrl.h"

namespace Ui {
class ConvoyerKeyboard;
}

class ConvoyerKeyboard : public QWidget {
    Q_OBJECT

   public:
    explicit ConvoyerKeyboard(std::shared_ptr<VNSim::BaseController> ptr,
                          QWidget *parent = nullptr);

    ~ConvoyerKeyboard();

   private slots:
    void on_pushButton_clicked();


   private:
    void sendMsg();
    struct ManualState {
        std::string selected = "";
    };
    ManualState manual_state_;

    Ui::ConvoyerKeyboard *ui;

    std::shared_ptr<VNSim::BaseController> model_ptr_;

    QTimer timer_;
};

#endif  // KEYBOARDFORM_H
