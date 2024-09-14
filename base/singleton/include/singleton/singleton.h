#ifndef __SINGLETON_H__
#define __SINGLETON_H__

#include <memory>

namespace VNSim{

template<typename T> 
class Singleton {
public:
    static std::shared_ptr<T> getInstance(){
        if(this_ptr == nullptr){
            this_ptr = std::make_shared<T>();
        }
        return this_ptr;
    }
    static void deleteInstance() {
        if (this_ptr != nullptr) {
            this_ptr.reset();
            this_ptr = nullptr;
        }
    }
private:
    Singleton(){}
    virtual ~Singleton(){}
private:
    static std::shared_ptr<T> this_ptr;
};

template<typename T>
std::shared_ptr<T> Singleton<T>::this_ptr = nullptr;
}
#endif