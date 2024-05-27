#ifndef ACTIONSERVICE_H
#define ACTIONSERVICE_H

#include "actionservice_global.h"
#include <string>
#include <thread>
#include "logvn/logvn.h"
#include "mqtt/client.h"
#include "thread"
#include "responsequeue.h"
using namespace std;
using namespace std::chrono;

namespace VNSim
{

constexpr auto RESPONSE_TOPIC	= mqtt::property::RESPONSE_TOPIC;
constexpr auto USER_PROPERTY	= mqtt::property::USER_PROPERTY;
constexpr auto CORRELATION_DATA	= mqtt::property::CORRELATION_DATA;


typedef string FUNC_NAME;
typedef string REPLY_TOPIC;


const int MQTT_QoS=0;

class ACTIONSERVICE_EXPORT ActionService
{
public:
    //parallel
    ActionService(string serverName, map<FUNC_NAME, function<void(REQUEST_JSONSTR,ResponseQueue *)> > &handles, Logvn &log, string host="tcp://localhost:1883");

    //serial
    ActionService(string serverName, map<FUNC_NAME, function<void(REQUEST_JSONSTR,ActionService *)> > &handles, Logvn &log, string host="tcp://localhost:1883");
    void sendOneResponse(RESPONSE_JSONSTR res);

    ~ActionService();

    void startServer()
    {
        std::thread t(&ActionService::run,this);
        t.detach();
    }

    void stopServer()
    {
        _isRunning=false;
    }


private:
    void run();
    bool init();
    bool reconnect();

    vector<FUNC_NAME> _funcNames;
    vector<int> _qos;
    map<FUNC_NAME, function<void(REQUEST_JSONSTR, ResponseQueue *)> > _handles;
    map<FUNC_NAME, function<void(REQUEST_JSONSTR, ActionService *)> > _handlesSerial;
    string _host;
    string _serverName;
    mqtt::client *_cli;
    std::string _replyTo;
    bool _isParallel;

    map<FUNC_NAME,ResponseQueue* > _outputContainer;
    bool _isRunning;
};

class ACTIONSERVICE_EXPORT ActionInvoker
{
public:
    ActionInvoker();
    ~ActionInvoker();

    bool init(FUNC_NAME funcName,Logvn &log,string host="tcp://localhost:1883");

    bool invoke(FUNC_NAME funcName, REQUEST_JSONSTR input,int64_t timeoutMillSeconds=20000);

    bool getReply(RESPONSE_JSONSTR &output,int64_t timeoutMillSeconds=20000);


private:
    void clear();


    mqtt::async_client * _cli;
    REPLY_TOPIC _replyTopic;
    FUNC_NAME _funcName;
    string _host;

};




}
#endif // ACTIONSERVICE_H
