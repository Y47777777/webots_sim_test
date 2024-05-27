#ifndef QSLEEP_H
#define QSLEEP_H
#include <QEventLoop>
#include <QTimer>
#include <QObject>
class QSleep : public QObject
{
    Q_OBJECT
public:
    explicit QSleep()
    {

    }

    static void sleep(const int msec=3000)
    {
        QEventLoop loop;
        QTimer timer;
        timer.setSingleShot(true);
        connect(&timer,SIGNAL(timeout()),&loop,SLOT(quit()));
        timer.start(msec);
        loop.exec();
    }

};

#endif
