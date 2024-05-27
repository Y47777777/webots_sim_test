#ifndef RESPONSEQUEUE_H
#define RESPONSEQUEUE_H

#include <string>
#include "boost/lockfree/queue.hpp"
using namespace std;

namespace VNSim
{

typedef string REQUEST_JSONSTR;
typedef string RESPONSE_JSONSTR;

struct OneResponse{
    RESPONSE_JSONSTR str;
};

class ResponseQueue
{

public:
    ResponseQueue()
    {
        _response=new boost::lockfree::queue<OneResponse* > (1024);
    }

    ~ResponseQueue()
    {
        OneResponse *r;
        while(_response->pop(r))
        {
            delete r;
        }
        delete[] _response;
    }

    bool pushBackOneResponse(RESPONSE_JSONSTR res)
    {
        OneResponse *r=new OneResponse;
        r->str=res;
        return _response->push(r);
    }

    bool popOneResponse(RESPONSE_JSONSTR &res)
    {
        OneResponse *r;
        if(_response->pop(r))
        {
            res=r->str;
            delete r;
            return true;
        }else
            return false;
    }

private:
    boost::lockfree::queue<OneResponse* > *_response;
};

}

#endif // RESPONSEQUEUE_H
