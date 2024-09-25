#ifndef __VOYERBELTMANAGER_H__
#define __VOYERBELTMANAGER_H__

#include <map>
#include <list>
#include <thread>
#include <semaphore.h>
#include <memory>
#include "time/time.h"
#include <webots/Node.hpp>
#include "geometry/geometry.h"
#include "singleton/singleton_soliddata.h"
#include "threadpool.h"

#define DEFAULT_THREAD_POOL 4

namespace VNSim{
    class VoyerBeltServer;
    class RingTimer;
    class VoyerBeltManager; 
    class VoyerBeltServer{
        private:
            int state_{-1}; // -1 is none, 0 is onPallet, 1 is offPallet
            int direction_ = {0}; // 0 is output, 1 is input 
            bool isOnTimer_{false};
            bool isOnStart_{true};
            bool isManual_{false};
            std::mutex isOnTimer_mutex_;
            std::string group_name_;
            std::list<Node*> input_pallets_;
            int time_ = {2000};
            Node* Server_;
            double s_trans_[3] = {0, 0, 0};
            double s_rotate[4] = {0, 0, 0, 0};
            double size_[3] = {1.05, 1.3, 1.3};
            double minX = 0;
            double maxX = 0;
            double minY = 0;
            double maxY = 0;
            double storage_offset_[3] = {0, 0, 0};
        public:
            VoyerBeltServer(){}
            ~VoyerBeltServer(){}
        public:
            void init(Node* node){
                input_pallets_.push_back(node);
                return;
            }
            void addPallet(){
                // TODO:
                if(!input_pallets_.empty()){
                    LOG_INFO("trigger add pallet --> %s, lX = %f lY = %f lZ = %f", group_name_.c_str(), s_trans_[0], s_trans_[1], size_[2]);
                    double trans[3] = { s_trans_[0] + storage_offset_[0], s_trans_[1] + storage_offset_[1], size_[2] + storage_offset_[2] };
                    input_pallets_.front()->getField("translation")->setSFVec3f(trans);
                    Singleton<PossibleSolidData>::getInstance()->addNode(input_pallets_.front());
                    input_pallets_.erase(input_pallets_.begin());
                }
                return;
            }
            void setBeltNode(Node* node){Server_ = node;
                time_ = Server_->getField("delay")->getSFInt32();
                const double* offset = Server_->getField("storage_offset")->getSFVec3f();
                direction_ = Server_->getField("direction")->getSFInt32();
                isManual_ = Server_->getField("manual")->getSFBool();
                if(time_ < 1000){
                    time_ = 1000;
                }
                const double* s_t = Server_->getPosition();
                const double* s_r = Server_->getOrientation();
                const double* table_size = Server_->getField("size")->getSFVec3f();
                const double* translation = Server_->getField("translation")->getSFVec3f();
                const double* rotation = Server_->getField("rotation")->getSFRotation();
                for(int i = 0; i < 3; i++){
                    s_trans_[i] = s_t[i];
                    size_[i] = table_size[i];
                    storage_offset_[i] = offset[i];
                }
                    
                Eigen::Matrix4d tran_matrix =
                    createTransformMatrix(rotation, translation);
                LOG_INFO("oX = %f, oY = %f, oZ = %f", translation[0], translation[1], translation[2]);
                std::vector<Eigen::Vector4d> p_list;
                p_list.push_back(
                    Eigen::Vector4d(size_[0] / 2, size_[1] / 2, size_[2], 1));
                p_list.push_back(
                    Eigen::Vector4d(-size_[0] / 2, size_[1] / 2, size_[2], 1));
                p_list.push_back(
                    Eigen::Vector4d(-size_[0] / 2, -size_[1] / 2, size_[2], 1));
                p_list.push_back(
                    Eigen::Vector4d(size_[0] / 2, -size_[1] / 2, size_[2], 1));
                for (int i = 0; i < p_list.size(); i++) {
                    p_list[i] = tran_matrix * p_list[i];
                    LOG_INFO("pointsX = %f, pointY = %f, pointZ = %f", p_list[i].x(), p_list[i].y(), p_list[i].z());
                    if(i == 0){
                        minX = p_list[i].x();
                        maxX = p_list[i].x();
                        minY = p_list[i].y();
                        maxY = p_list[i].y();
                    }
                    if(p_list[i].x() < minX){
                        minX = p_list[i].x();
                    }else if(p_list[i].x() > minX){
                        maxX = p_list[i].x();
                    }

                    if(p_list[i].y() < minY){
                        minY = p_list[i].y();
                    }else if(p_list[i].y() > maxY){
                        maxY = p_list[i].y();
                    }
                }
                // LOG_INFO("i = %d,  %f,  %f,  %f", i, p_list[i].x(),
                //          p_list[i].y(), p_list[i].z());
                LOG_INFO("minX = %f, maxX = %f, minY = %f, maxY = %f", minX, maxX, minY, maxY);
            }
            std::string getGroup(){return group_name_;}
            void setGroup(std::string group){group_name_ = group;}
            int getState(){return state_;}
            void setOnStart(bool onStart){isOnStart_ = onStart;}
            bool isOnStart(){return isOnStart_;}
            void removePallet(){
                static double more = 0;
                static double base = 100;
                std::list<Node*> l_node_list;
                Singleton<PossibleSolidData>::getInstance()->getNode(l_node_list);
                for(const auto &it: l_node_list){
                    const double* w_pos = it->getPosition();
                    if( ((minX - 0.002 < w_pos[0]) && (w_pos[0] < maxX + 0.002)) && ((minY - 0.002 < w_pos[1]) && (w_pos[1] < maxY + 0.002))){
                        // x, y in range, possibly no Z
                        Field* field = it->getField("translation");
                        if(field != nullptr){
                            const double* o_trans = field->getSFVec3f();
                            double trans[3] = {o_trans[0], o_trans[1] + base + more, o_trans[2]};
                            field->setSFVec3f(trans);
                            more += 1;
                        }
                        LOG_INFO("remove name = %s, x = %f, y = %f, x1 = %f, y1 = %f", it->getField("name")->getSFString().c_str(), w_pos[0], w_pos[1], s_trans_[0], s_trans_[1]);
                    }
                }
                return;
            }
            bool setOnTimer(bool flag){
                std::unique_lock<std::mutex> lock(isOnTimer_mutex_);
                //printf("%s --> set onTimer = %d, isOnTimer = %d\n", group_name_.c_str(), flag, isOnTimer_);
                bool result = true;
                if(flag == true && isOnTimer_ == true){
                    result = false;
                }
                isOnTimer_ = flag;
                return result;
            }
            bool isCorrectDirection(uint32_t direction){
                return direction_ == direction;
            }
            bool isManual(){return isManual_;}
            int getDelayTime(){return time_;}
            void onRemoveCallback(){
                this->removePallet();
                this->setOnTimer(false);
            }
            void onAddCallback(){
                this->addPallet();
                this->setOnTimer(false);
            }
    };
    class RingTimer{
        public:
            struct Msg{
                std::shared_ptr<VoyerBeltServer> server;
                std::chrono::high_resolution_clock::time_point start;
                int event; // 0 add pallet, 1 remove pallet
                uint64_t trigger_time;
            };
        private:
            std::unique_ptr<ThreadPool> th_pool_ptr_;
            std::thread palletRemoveTimer_;
            std::mutex msg_list_mutex_;
            bool running_{true};
            std::list<Msg> msg_list_;
        private:
            void run(){
                Timer alarm;
                while(running_){
                    alarm.alarmTimerInit(1000);
                    alarm.wait();
                    this->removeMsg();
                }
            }
            void triggerEvent(int event, std::shared_ptr<VoyerBeltServer> ptr){
                if(event == 0)
                    th_pool_ptr_->enqueue(std::bind(&VoyerBeltServer::onAddCallback, ptr));
                else
                    th_pool_ptr_->enqueue(std::bind(&VoyerBeltServer::onRemoveCallback, ptr));
            }
        public:
            void init(){
                th_pool_ptr_ = std::make_unique<ThreadPool>(DEFAULT_THREAD_POOL);
                std::thread l_thread(std::bind(&RingTimer::run, this));
                palletRemoveTimer_ = std::move(l_thread);
            }
            void addMsg(std::shared_ptr<VoyerBeltServer> ptr, int event, const VoyerBeltManager* ptr2, uint64_t time = 2000){
                std::unique_lock<std::mutex> lock(msg_list_mutex_);
                msg_list_.push_back({.server = ptr, .start = std::chrono::high_resolution_clock::now(), \
                                     .event = event, .trigger_time = time});
            }
            void removeMsg(){
                bool trigger_next = true;
                std::shared_ptr<VoyerBeltServer> ptr = nullptr;
                std::list<std::list<Msg>::iterator> remove_it_list;
                std::list<Msg> call_back_list;
                std::list<Msg>::iterator b_it;
                std::list<std::list<Msg>::iterator>::iterator rb_it;
                {
                    std::unique_lock<std::mutex> lock(msg_list_mutex_);
                    for(b_it = msg_list_.begin(); b_it != msg_list_.end(); b_it++){
                        auto end = std::chrono::high_resolution_clock::now();
                        auto diff = std::chrono::duration_cast<milliseconds>(end - b_it->start);
                        LOG_DEBUG("high resolution duration = %lu ms", diff);
                        if(diff >= (std::chrono::milliseconds(b_it->trigger_time) )){
                            call_back_list.push_back(*b_it);
                            remove_it_list.push_back(b_it);
                        }
                    }
                    for(rb_it = remove_it_list.begin(); rb_it != remove_it_list.end(); rb_it++){
                        msg_list_.erase(*rb_it);
                    }
                }
                for(auto &it: call_back_list){
                    this->triggerEvent(it.event, it.server);
                }
            }
            void trigger(int event, std::shared_ptr<VoyerBeltServer> ptr){
                this->triggerEvent(event, ptr);
            }
    };
    class VoyerBeltManager{
        private:
            std::map<std::string, std::shared_ptr<VoyerBeltServer>> belts_;
            RingTimer timer_;
        public:
            VoyerBeltManager(){
                timer_.init();
            }
            ~VoyerBeltManager(){}
        public:
            void addBelt(std::string belt, std::string group, Node* server){
                if(belts_.find(belt) == belts_.end()){
                    std::shared_ptr ptr = std::make_shared<VoyerBeltServer>();
                    ptr->setGroup(group);
                    ptr->setBeltNode(server);
                    belts_[belt] = ptr;
                }
            }
            void initPallets(std::string groups, Node* node){
                for(const auto& [key, value] : belts_){
                    if(value->getGroup() == groups){
                        value->init(node); 
                    }
                }
            }
            void addPossibleNode(Node* node){
                Singleton<PossibleSolidData>::getInstance()->addNode(node);
            }
            void removeBelt(std::string belt){
                belts_.erase(belt);
            }
            int addRemovePallet(std::string belt, bool isManual = false, uint32_t who = 0){
                int ret = -1;
                auto it = belts_.find(belt);
                if(it != belts_.end()){
                    do{
                        ret = it->second->isOnStart() ? 1 : 0;
                        if(ret && (who == 1)){
                            it->second->setOnStart(false);
                        }
                        if(isManual != it->second->isManual())break;
                        if(!it->second->isCorrectDirection(1))break;
                        if(isManual){
                            LOG_INFO("trigger remove pallet immediately --> %s", belt.c_str());
                            timer_.trigger(1, it->second);
                        }else{
                            int delay = it->second->getDelayTime();
                            if(it->second->setOnTimer(true)){
                                LOG_INFO("trigger remove pallet timer --> %s, delay = %d ms", belt.c_str(), delay);
                                timer_.addMsg(it->second, 1, this, delay);
                            }
                        }
                    }while(0);
                }
                return ret;
            }
            int addRandomPallet(std::string belt, bool isManual = false, bool isDelay = true, uint32_t who = 0){
                int ret = -1;
                auto it = belts_.find(belt);
                if(it != belts_.end()){
                    do{
                        ret = it->second->isOnStart() ? 1 : 0;
                        if(!isManual && ret && (who == 1)){
                            it->second->setOnStart(false);
                        }
                        if(isManual != it->second->isManual())break;
                        if(!it->second->isCorrectDirection(0))break;
                        if(!isDelay){
                            LOG_INFO("trigger add pallet immediately --> %s", belt.c_str());
                            timer_.trigger(0, it->second);
                        }
                        else{
                            int delay = it->second->getDelayTime();
                            if(it->second->setOnTimer(true)){
                                LOG_INFO("trigger add pallet timer --> %s, delay = %d ms", belt.c_str(), delay);
                                //printf("trigger add pallet timer --> %s, delay = %d ms\n", belt.c_str(), delay);
                                timer_.addMsg(it->second, 0, this, delay);
                            }
                        }
                    }while(0);
                }
                return ret;
            }
            int getServerList(std::list<std::string>& server_list){
                std::map<std::string, std::shared_ptr<VoyerBeltServer>>::const_iterator it;
                int ret = 0;
                for(it = belts_.begin(); it != belts_.end(); it++) server_list.push_back(it->first);
                return ret;
            }
    };
}

#endif