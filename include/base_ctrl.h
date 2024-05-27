#ifndef __BASE_CTRL_H__
#define __BASE_CTRL_H__

#include <vector>
#include <QThread>

class BaseController : public QThread {
   protected:
    bool isManual_{false};

   public:
    explicit BaseController(QObject *parent = nullptr) : QThread{parent} {}
    virtual ~BaseController() {}
    void shiftControlMode(bool mode) { isManual_ = mode; }
    void run() { this->runSimulation(); }

   public:
    // change to none pure
    virtual int init() = 0;

    virtual void runSimulation() = 0;

   public:
    // change to none pure
    /*Manual Contral*/
    virtual void onManualMsg(const char *msg) = 0;

    virtual void onManualReport(std::string &msg) = 0;

    /*Auto Contral*/
    virtual void onRemoteMsg(uint8_t *msg, int len) = 0;
};

#endif
