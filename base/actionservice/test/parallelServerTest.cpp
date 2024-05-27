#include "actionservice/actionservice.h"
// #include "logvn/logvn.h"
#include "nlohmann/json.hpp"
using namespace std;
using namespace VNSim;


class ServerHandle
{
public:
    ServerHandle() {}

    static void addFunctionHandle(REQUEST_JSONSTR input, ResponseQueue *rqueue)
    {
        LOG_DEBUG("input:%s",input.c_str());
        nlohmann::json in=nlohmann::json::parse(input);

        if(in.contains("x")&&in.contains("y"))
        {
            int x=in["x"];
            int y=in["y"];
            int r=x+y;

            // nlohmann::json mid_result;
            // for(int i=0;i<2;i++)
            // {
            //     mid_result["Status"]="transcating";
            //     mid_result["Rate"]=to_string(100*i/2);

            //     string mid_result_str=mid_result.dump();
            //     rqueue->pushBackOneResponse(mid_result_str);

            //     //this_thread::sleep_for(chrono::milliseconds(100));
            // }

            nlohmann::json result;
            result["Result"]=to_string(r);
            result["Status"]="success";
            string result_str=result.dump();
            rqueue->pushBackOneResponse(result_str);
        }
        else
        {
            nlohmann::json result;
            result["ErrorTips"]="Input not contains x or y ";
            result["ErrorType"]=-1;
            result["Status"]="failed";
            string result_str=result.dump();
            rqueue->pushBackOneResponse(result_str);
        }

    }

    static void multFunctionHandle(REQUEST_JSONSTR input,  ResponseQueue *rqueue)
    {
        LOG_DEBUG("input:%s",input.c_str());
        nlohmann::json in=nlohmann::json::parse(input);

        if(in.contains("x")&&in.contains("y"))
        {
            int x=in["x"];
            int y=in["y"];
            int r=x*y;

            nlohmann::json mid_result;
            for(int i=0;i<2;i++)
            {
                mid_result["Status"]="transcating";
                mid_result["Rate"]=to_string(100*i/2);

                string mid_result_str=mid_result.dump();
                rqueue->pushBackOneResponse(mid_result_str);

                //this_thread::sleep_for(chrono::milliseconds(100));
            }

            nlohmann::json result;
            result["Result"]=to_string(r);
            result["Status"]="success";
            string result_str=result.dump();
            rqueue->pushBackOneResponse(result_str);
        }
        else
        {
            nlohmann::json result;
            result["ErrorTips"]="Input not contains x or y ";
            result["ErrorType"]=-1;
            result["Status"]="failed";
            string result_str=result.dump();
            rqueue->pushBackOneResponse(result_str);
        }
    }


    static void addFunctionHandleSerial(REQUEST_JSONSTR input,ActionService* server)
    {
        LOG_DEBUG("input:%s",input.c_str());
        nlohmann::json in=nlohmann::json::parse(input);

        if(in.contains("x")&&in.contains("y"))
        {
            int x=in["x"];
            int y=in["y"];
            int r=x+y;

            // nlohmann::json mid_result;
            // for(int i=0;i<2;i++)
            // {
            //     mid_result["Status"]="transcating";
            //     mid_result["Rate"]=to_string(100*i/2);

            //     string mid_result_str=mid_result.dump();
            //     server->sendOneResponse(mid_result_str);

            //     //this_thread::sleep_for(chrono::milliseconds(100));
            // }

            //this_thread::sleep_for(chrono::milliseconds(300));

            static int count=0;
            nlohmann::json result;
            result["Count"]=count;
            result["Result"]=to_string(r);
            result["Status"]="success";
            string result_str=result.dump();
            server->sendOneResponse(result_str);

            count++;
        }
        else
        {
            nlohmann::json result;
            result["ErrorTips"]="Input not contains x or y ";
            result["ErrorType"]=-1;
            result["Status"]="failed";
            string result_str=result.dump();
            server->sendOneResponse(result_str);
        }
    }

    static void multFunctionHandleSerial(REQUEST_JSONSTR input,ActionService* server)
    {
        LOG_DEBUG("input:%s",input.c_str());
        nlohmann::json in=nlohmann::json::parse(input);

        if(in.contains("x")&&in.contains("y"))
        {
            int x=in["x"];
            int y=in["y"];
            int r=x*y;

            // nlohmann::json mid_result;
            // for(int i=0;i<2;i++)
            // {
            //     mid_result["Status"]="transcating";
            //     mid_result["Rate"]=to_string(100*i/2);

            //     string mid_result_str=mid_result.dump();
            //     server->sendOneResponse(mid_result_str);

            //     //this_thread::sleep_for(chrono::milliseconds(100));
            // }

            nlohmann::json result;
            result["Result"]=to_string(r);
            result["Status"]="success";
            string result_str=result.dump();
            server->sendOneResponse(result_str);
        }
        else
        {
            nlohmann::json result;
            result["ErrorTips"]="Input not contains x or y ";
            result["ErrorType"]=-1;
            result["Status"]="failed";
            string result_str=result.dump();
            server->sendOneResponse(result_str);
        }
    }

};




int main()
{
    g_Logger.initLog("../paralle_server_test.ini");


    map<FUNC_NAME, function<void(REQUEST_JSONSTR, ResponseQueue *)> > _handles;
    std::function<void(REQUEST_JSONSTR, ResponseQueue *)> addHandle=ServerHandle::addFunctionHandle;
    std::function<void(REQUEST_JSONSTR, ResponseQueue *)> multHandle=ServerHandle::multFunctionHandle;
    _handles["vision/paralleservice/add"]=addHandle;
    _handles["vision/paralleservice/mult"]=multHandle;
    ActionService serverparalle("paralle_service_test",_handles,g_Logger);

    serverparalle.startServer();


    map<FUNC_NAME, function<void(REQUEST_JSONSTR,ActionService*)> > _handlesSerial;
    std::function<void(REQUEST_JSONSTR,ActionService*)> addHandleSerial=ServerHandle::addFunctionHandleSerial;
    std::function<void(REQUEST_JSONSTR,ActionService*)> multHandleSerial=ServerHandle::multFunctionHandleSerial;
    _handlesSerial["vision/serialservice/add"]=addHandleSerial;
    _handlesSerial["vision/serialservice/mult"]=multHandleSerial;
    ActionService serverserial("serial_service_test",_handlesSerial,g_Logger);

    serverserial.startServer();

    while(true)
    {
        this_thread::sleep_for(chrono::milliseconds(1000));
    }

    return 0;
}
