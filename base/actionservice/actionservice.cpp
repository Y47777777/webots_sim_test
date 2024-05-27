#include "actionservice.h"

namespace VNSim
{
ActionService::ActionService(string serverName, map<FUNC_NAME, function<void (REQUEST_JSONSTR, ResponseQueue *)> > &handles, Logvn &log, string host)
{
    _host=host;
    _serverName=serverName;
    _cli=NULL;
    _isParallel=true;
    _isRunning=false;

    map<FUNC_NAME, function<void (REQUEST_JSONSTR, ResponseQueue *)> >::iterator itr;
    for(itr = handles.begin(); itr != handles.end(); itr++)
    {
        _qos.push_back(MQTT_QoS);
        _funcNames.push_back(itr->first);
        _handles[itr->first]=itr->second;
    }
}

ActionService::ActionService(string serverName, map<FUNC_NAME, function<void (REQUEST_JSONSTR, ActionService *)> > &handles, Logvn &log, string host)
{
    _host=host;
    _serverName=serverName;
    _cli=NULL;
    _isParallel=false;
    _isRunning=false;

    map<FUNC_NAME, function<void (REQUEST_JSONSTR, ActionService *)> >::iterator itr;
    for(itr = handles.begin(); itr != handles.end(); itr++)
    {
        _qos.push_back(MQTT_QoS);
        _funcNames.push_back(itr->first);
        _handlesSerial[itr->first]=itr->second;
    }
}

ActionService::~ActionService()
{
    if(_cli)
        delete _cli;

    if(!_outputContainer.empty())
    {
        map<FUNC_NAME,ResponseQueue* >::iterator itr;
        for(itr = _outputContainer.begin(); itr != _outputContainer.end(); itr++)
        {
            delete itr->second;
        }
    }
}

bool ActionService::reconnect()
{
    constexpr int N_ATTEMPT = 30;

    for (int i=0; i<N_ATTEMPT && !_cli->is_connected(); ++i) {
        try {
            _cli->reconnect();
            return true;
        }
        catch (const mqtt::exception&) {
            this_thread::sleep_for(seconds(1));
        }
    }
    return false;
}

bool ActionService::init()
{
    if(_cli)
        return true;

    mqtt::create_options createOpts(MQTTVERSION_5);
    _cli=new mqtt::client(_host, _serverName, createOpts);
    _cli->start_consuming();

    auto connOpts = mqtt::connect_options_builder()
                        .mqtt_version(MQTTVERSION_5)
                        .keep_alive_interval(std::chrono::seconds(20))
                        .clean_start(true)
                        .finalize();
    try
    {
        _cli->connect(connOpts);
        _cli->subscribe(_funcNames, _qos);
        LOG_DEBUG("Connected to the MQTT server...");
        return true;
    }catch (const mqtt::exception& exc)
    {

        LOG_ERROR("Service error:%s,",exc.what());
        return false;
    }
}

void ActionService::sendOneResponse(RESPONSE_JSONSTR res)
{
    if(_isParallel)
        return;

    mqtt::properties props {
                           { mqtt::property::RESPONSE_TOPIC, _replyTo },
                           };
    auto reply_msg = mqtt::message::create(_replyTo, res, MQTT_QoS, false, props);
    _cli->publish(reply_msg);
    //LOG_DEBUG("reply:%s result:%s ",replyTo.c_str(),output.c_str());
}

void ActionService::run()
{
    if(_isRunning)
        return;
    while(!init())
    {
        delete _cli;
        _cli=NULL;
        this_thread::sleep_for(seconds(1));
    }
    _isRunning=true;
    while(_isRunning)
    {
        mqtt::const_message_ptr msg;
        if(_cli->try_consume_message(&msg))
        {
            //LOG_DEBUG("Consume a message...");

            const mqtt::properties& props = msg.get()->get_properties();
            if (props.contains(RESPONSE_TOPIC))
            {
                string replyTo = mqtt::get<string>(props, RESPONSE_TOPIC);
                _replyTo=replyTo;
                //LOG_DEBUG("Client wants a reply to %s",replyTo.c_str());

                REQUEST_JSONSTR input=msg.get()->to_string();

                if(_outputContainer.find(replyTo)==_outputContainer.end())
                {
                    ResponseQueue *rq=new ResponseQueue;
                    _outputContainer[replyTo]=rq;
                }

                if(_isParallel)
                {
                    std::thread t(_handles[msg.get()->get_topic()],input,_outputContainer[replyTo]);
                    t.detach();
                }else
                {
                    _handlesSerial[msg.get()->get_topic()](input,this);
                }

            }
            else
            {
                LOG_ERROR("Msg props not contains RESPONSE_TOPIC !");
            }
        }else
        {
            //LOG_DEBUG("no msg...");
            if (!_cli->is_connected())
            {
                LOG_WARN("Lost connection. Attempting reconnect" );
                if (reconnect())
                {
                    _cli->subscribe(_funcNames, _qos);
                    LOG_INFO("Reconnected...");
                    continue;
                }
                else
                {
                    LOG_ERROR("Reconnect failed. Retry...");
                    this_thread::sleep_for(seconds(1));
                    continue;
                }
            }

        }

        if(_isParallel)
        {
            map<FUNC_NAME,ResponseQueue* >::iterator itr;
            for(itr = _outputContainer.begin(); itr != _outputContainer.end(); itr++)
            {
                string replyTo=itr->first;
                REQUEST_JSONSTR output;
                while(itr->second->popOneResponse(output))
                {
                    mqtt::properties props {
                                           { mqtt::property::RESPONSE_TOPIC, replyTo },
                                           };
                    auto reply_msg = mqtt::message::create(replyTo, output, MQTT_QoS, false, props);
                    _cli->publish(reply_msg);
                    //LOG_DEBUG("reply:%s result:%s ",replyTo.c_str(),output.c_str());
                }
            }
        }

        this_thread::sleep_for(milliseconds(1));
    }

}



ActionInvoker::ActionInvoker()
{
    _cli=NULL;
}
ActionInvoker::~ActionInvoker()
{
    clear();
}

void ActionInvoker::clear()
{
    if(_cli!=NULL)
    {
        if(_cli->is_connected())
        {
            _cli->unsubscribe(_replyTopic)->wait();
            _cli->disconnect()->wait();
        }
        delete _cli;
        _cli=NULL;
    }
}

bool ActionInvoker::init(FUNC_NAME funcName, Logvn &log, string host)
{
    _host=host;
    _funcName=funcName;
    clear();

    mqtt::create_options createOpts(MQTTVERSION_5);
    _cli=new mqtt::async_client(_host, "", createOpts);
    _cli->start_consuming();

    mqtt::token_ptr tok;

    //LOG_DEBUG("Connecting...");
    auto connOpts = mqtt::connect_options_builder()
                        .mqtt_version(MQTTVERSION_5)
                        .clean_start()
                        .finalize();
    tok = _cli->connect(connOpts);

    auto connRsp = tok->get_connect_response();

    string clientId = get<string>(connRsp.get_properties(),
                                  mqtt::property::ASSIGNED_CLIENT_IDENTIFER);

    _replyTopic = "replies/"+ funcName +"/"+ clientId;

    tok = _cli->subscribe(_replyTopic.c_str(), MQTT_QoS);
    tok->wait();

    if (int(tok->get_reason_code()) != MQTT_QoS) {
        LOG_ERROR("Server doesn't support reply QoS:%d",MQTT_QoS);
        return false;
    }

    return true;
}

bool ActionInvoker::getReply(RESPONSE_JSONSTR &output, int64_t timeoutMillSeconds)
{
    auto msg = _cli->try_consume_message_for(std::chrono::milliseconds(timeoutMillSeconds));
    if (!msg)
    {
        if(!_cli->is_connected())
        {
            LOG_WARN("mqtt disconnected , please reinit！ ");
            return false;
        }
        LOG_WARN("Didn't receive a reply from server. time out = %d ms ",timeoutMillSeconds);
        return false;
    }

    output=msg->to_string();
    return true;
}

bool ActionInvoker::invoke(FUNC_NAME funcName, REQUEST_JSONSTR input, int64_t timeoutMillSeconds)
{
    if(funcName==_funcName)
    {
        if(input.size()>0)
        {
        mqtt::properties props {
            { mqtt::property::RESPONSE_TOPIC, _replyTopic },
        };

        auto pubmsg = mqtt::message_ptr_builder()
                          .topic(funcName)
                          .payload(input.c_str())
                          .qos(MQTT_QoS)
                          .properties(props)
                          .finalize();

        bool ret= _cli->publish(pubmsg)->wait_for(std::chrono::milliseconds(timeoutMillSeconds));
        if(ret)
        {
            return true;
        }
        else
        {
            if(!_cli->is_connected())
            {
                LOG_WARN("mqtt disconnected , please reinit！ ");
                return false;
            }
            LOG_WARN("publish failed , timeoutSeconds=%d s ",timeoutMillSeconds);
            return false;
        }
        }else
        {
            LOG_WARN("input is empty!");
            return false;
        }

    }
    else
    {
        LOG_WARN("This invoker is inited for %s , funcName is wrong!", _funcName.c_str());
        return false;
    }

}










































}
