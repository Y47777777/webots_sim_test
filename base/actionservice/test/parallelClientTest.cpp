#include <iostream>
#include "actionservice/actionservice.h"
#include "logvn/logvn.h"
#include "nlohmann/json.hpp"
using namespace std;
using namespace VNSim;

using namespace std;

#include <ctime>

int main()
{
    g_Logger.initLog("../paralle_client_test.ini");

    ActionInvoker addInvoker;
    bool ret=addInvoker.init("vision/paralleservice/add",g_Logger);
    LOG_DEBUG("init addInvoker %d",ret);

    ActionInvoker multInvoker;
    ret=multInvoker.init("vision/paralleservice/mult",g_Logger);
    LOG_DEBUG("init multInvoker %d",ret);


    nlohmann::json input;
    input["x"]=20;
    input["y"]=30;

    time_t t=time(0);

    int count=0;
    while(true)
    {
        REQUEST_JSONSTR inputStr=input.dump();
        if(addInvoker.invoke("vision/paralleservice/add",inputStr))
        {
            LOG_DEBUG("inputStr:%s",inputStr.c_str());
            while(true)
            {
                RESPONSE_JSONSTR outputStr;
                if(addInvoker.getReply(outputStr))
                {
                    nlohmann::json output=nlohmann::json::parse(outputStr);
                    if(output["Status"]=="success")
                    {
                        LOG_DEBUG("%s",outputStr.c_str());
                        break;
                    }
                    else if(output["Status"]=="failed")
                    {
                        string erroeMsg;
                        output["ErrorTips"].get_to(erroeMsg);
                        int errorType=0;
                        output["ErrorType"].get_to(errorType);
                        LOG_DEBUG("ErrorTips:%s ErrorType:%d",erroeMsg.c_str(),errorType);
                        break;
                    }
                    else
                    {
                        LOG_DEBUG("%s",outputStr.c_str());
                    }
                }else
                    break;
            }
        }
        count++;
        if(count==1)
            break;
    }

    LOG_DEBUG("cost:%d",time(0)-t);


    return 0;
}
