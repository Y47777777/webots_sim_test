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

namespace VNSim{
    class VoyerBeltServer;
    class RingTimer;
    class VoyerBeltManager; 
    class VoyerBeltServer{
        private:
            int state_{-1};
            bool isOnTimer_{false};
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
        public:
            VoyerBeltServer(){}
            ~VoyerBeltServer(){}
        public:
            void init(Node* node){
                printf("%s --> add Pallet\n", group_name_.c_str());
                input_pallets_.push_back(node);
                return;
            }
            void addPallet(){
                // TODO:
                std::cout << __FUNCTION__ << std::endl;
                if(!input_pallets_.empty()){
                    double trans[3] = { s_trans_[0], s_trans_[1], 1.31};
                    input_pallets_.front()->getField("translation")->setSFVec3f(trans);
                    input_pallets_.erase(input_pallets_.begin());
                }
                return;
            }
            void setBeltNode(Node* node){Server_ = node;
                time_ = Server_->getField("disapper")->getSFInt32();
                if(time_ < 1000){
                    time_ = 1000;
                }
                const double* s_t = Server_->getPosition();
                const double* s_r = Server_->getOrientation();
                const double* table_size = Server_->getField("size")->getSFVec3f();
                const double* translation = Server_->getField("translation")->getSFVec3f();
                const double* rotation = Server_->getField("rotation")->getSFRotation();
                for(int i = 0; i < 3; i++)
                    s_trans_[i] = s_t[i];
                for(int i = 0; i < 2; i++){
                    size_[i] = table_size[i];
                    printf("table = %f\n", table_size[i]);
                }
                    
                Eigen::Matrix4d tran_matrix =
                    createTransformMatrix(rotation, translation);
                printf("oX = %f, oY = %f, oZ = %f\n", translation[0], translation[1], translation[2]);
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
                    printf("pointsX = %f, pointY = %f, pointZ = %f\n", p_list[i].x(), p_list[i].y(), p_list[i].z());
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
                printf("minX = %f, maxX = %f, minY = %f, maxY = %f\n", minX, maxX, minY, maxY);
            }
            std::string getGroup(){return group_name_;}
            void setGroup(std::string group){group_name_ = group;}
            int getState(){return state_;}
            void removePallet(){
                std::cout << __FUNCTION__ << std::endl;
                static double more = 0;
                static double base = 100;
                const std::list<Node*>* l_node_list = Singleton<PossibleSolidData>::getInstance()->getNode();
                if(l_node_list->empty()){
                    printf("BAD, empty list...\n");
                }
                for(const auto &it: *l_node_list){
                    const double* w_pos = it->getPosition();
                    // if((std::fabs(w_pos[0] - s_trans_[0]) < 1.05/2) &&
                    //     (std::fabs(w_pos[1] - s_trans_[1]) < 1.3/2)){
                    if( ((minX - 0.002 < w_pos[0]) && (w_pos[0] < maxX + 0.002)) && ((minY - 0.002 < w_pos[1]) && (w_pos[1] < maxY + 0.002))){
                        // x, y in range, possibly no Z
                        Field* field = it->getField("translation");
                        if(field != nullptr){
                            const double* o_trans = field->getSFVec3f();
                            double trans[3] = {o_trans[0], o_trans[1] + base + more, o_trans[2]};
                            field->setSFVec3f(trans);
                            more += 1;
                        }
                        printf("remove name = %s, x = %f, y = %f, x1 = %f, y1 = %f\n", it->getField("name")->getSFString().c_str(), w_pos[0], w_pos[1], s_trans_[0], s_trans_[1]);
                    }
                }
                return;
            }
            bool setOnTimer(bool flag){
                std::unique_lock<std::mutex> lock(isOnTimer_mutex_);
                bool result = true;
                if(flag == true && isOnTimer_ == true){
                    result = false;
                }
                isOnTimer_ = flag;
                return result;
            }
    };
    class RingTimer{
        public:
            struct Msg{
                std::shared_ptr<VoyerBeltServer> server;
                std::chrono::high_resolution_clock::time_point start;
                uint64_t trigger_time;
            };
        private:
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
        public:
            void init(){
                // shadow_manager_ = l_manager;
                std::thread l_thread(std::bind(&RingTimer::run, this));
                palletRemoveTimer_ = std::move(l_thread);
            }
            void addMsg(std::shared_ptr<VoyerBeltServer> ptr, const VoyerBeltManager* ptr2, uint64_t time = 2000){
                std::unique_lock<std::mutex> lock(msg_list_mutex_);
                msg_list_.push_back({.server = ptr, .start = std::chrono::high_resolution_clock::now(), .trigger_time = time});
            }
            void removeMsg(){
                bool trigger_next = true;
                std::shared_ptr<VoyerBeltServer> ptr = nullptr;
                std::list<std::list<Msg>::iterator> remove_it_list;
                std::list<std::shared_ptr<VoyerBeltServer>> call_back_list;
                std::list<Msg>::iterator b_it;
                std::list<std::list<Msg>::iterator>::iterator rb_it;
                {
                    std::unique_lock<std::mutex> lock(msg_list_mutex_);
                    for(b_it = msg_list_.begin(); b_it != msg_list_.end(); b_it++){
                        auto end = std::chrono::high_resolution_clock::now();
                        auto diff = std::chrono::duration_cast<milliseconds>(end - b_it->start);
                        printf("high resolution duration = %lu ms\n", diff);
                        if(diff >= (std::chrono::milliseconds(b_it->trigger_time) )){
                            printf("push_back...\n");
                            call_back_list.push_back(b_it->server);
                            remove_it_list.push_back(b_it);
                        }
                    }
                    for(rb_it = remove_it_list.begin(); rb_it != remove_it_list.end(); rb_it++){
                        printf("Erase...\n");
                        msg_list_.erase(*rb_it);
                    }
                }
                for(auto &it: call_back_list){
                    it->removePallet();
                    it->setOnTimer(false);
                }
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
                    printf("belt = %s, group = %s\n", belt.c_str(), group.c_str());
                    std::shared_ptr ptr = std::make_shared<VoyerBeltServer>();
                    ptr->setGroup(group);
                    ptr->setBeltNode(server);
                    belts_[belt] = ptr;
                }
            }
            void initPallets(std::string groups, Node* node){
                printf("groups = %s\n", groups.c_str());
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
            void addRemovePallet(std::string belt){
                auto it = belts_.find(belt);
                if(it != belts_.end()){
                    if(it->second->setOnTimer(true)){
                        std::cout << "add timer...." << std::endl;
                        timer_.addMsg(it->second, this);
                    }
                }
            }
            void addRandomPallet(std::string belt){
                auto it = belts_.find(belt);
                if(it != belts_.end()){
                    it->second->addPallet();
                }
            }
    };
}

#endif