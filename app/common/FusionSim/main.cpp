#include <thread>
#include "ecal_wrapper/ecal_wrapper.h"
#include "sim_data_flow/voyerbelt_msg.pb.h"
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Lidar.hpp>
#include <webots/Node.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Supervisor.hpp>
#include "sim_data_flow/point_cloud.pb.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream> 
#include <fstream> 
#include <sys/stat.h>

using namespace VNSim;
using namespace webots;

std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;

std::string getTime()
{
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S",localtime(&timep));
    return tmp;
}


int main(int argc, char *argv[]){

    std::string log_file = "testlog.txt";
    std::ofstream myfile(log_file, std::ios::out|std::ios::app); 
    
    Robot *robot = new Robot();
    Camera *camera = robot->getCamera("camera");
    camera->enable(50);
    camera->recognitionEnable(50);
    camera->enableRecognitionSegmentation();
    myfile << "camera start..." << std::endl;

    RangeFinder *rangefinder = robot->getRangeFinder("range-finder");
    rangefinder->enable(50);
    myfile << "rangefinder start..." << std::endl;

    Lidar *lidar = robot->getLidar("lidar_0");
    lidar->enable(100);
    lidar->enablePointCloud();
    myfile << "lidar start..." << std::endl;

    Keyboard *keyboard = robot->getKeyboard();
    keyboard->enable(10);
    myfile << "keyboard start..." << std::endl;

    Supervisor *supervisor = Supervisor::getSupervisorInstance();
    Node *robot_node = supervisor->getFromDef("RobotNode");

    myfile << "simulation start..." << std::endl;
    myfile.close();

    // const std::vector<std::string> dir_list = {"rgb", "seg", "depth", "pcd"};
    // for(auto dir : dir_list)
    // {
    //     if(opendir(dir.c_str()) <= 0)
    //     {
    //         mkdir(dir.c_str(), 777);
    //     }
    // }

    while(robot->step(10) != -1)
    {
        int get_key = keyboard->getKey();

        // std::ofstream myfile(log_file, std::ios::out|std::ios::app); 
        // myfile << "get_key = " << get_key << std::endl;
        // myfile.close();
        
        if(get_key == 's' || get_key == 'S')
        {
            std::string time = getTime();
            const std::string save_path = time + ".png";
            camera->saveImage(save_path.c_str(), 10);

            const std::string save_path_seg = time + "-seg.png";
            camera->saveRecognitionSegmentationImage(save_path_seg.c_str(), 10);

            const std::string save_path_range = time + "-range.png";
            rangefinder->saveImage(save_path_range.c_str(), 10);

            const webots::LidarPoint *point_cloud = lidar->getPointCloud();
            int point_num = lidar->getNumberOfPoints();

            std::ofstream myfile(log_file, std::ios::out|std::ios::app); 
            myfile << "point_num = " << point_num << " save_path = " << save_path << std::endl;
            myfile.close();

            if(point_num >= 0)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

                for(int i = 0; i < point_num; ++i) 
                {
                    double x = point_cloud[i].x;
                    double y = point_cloud[i].y;
                    double z = point_cloud[i].z;

                    // std::ofstream myfile(log_file, std::ios::out|std::ios::app); 
                    // myfile << " x = " << x << " y = " << y << " z = " << z << std::endl;
                    // myfile.close();

                    if (std::abs(x) != INFINITY && std::abs(y) != INFINITY && std::abs(z) != INFINITY)
                    {
                        pcl::PointXYZ point;
                        point.x = x;
                        point.y = y;
                        point.z = z;
                        cloud_pcl->points.push_back(point);
                    }
                }
                std::ofstream myfile(log_file, std::ios::out|std::ios::app); 
                myfile << "cloud_pcl.points.size = " << cloud_pcl->points.size() << std::endl;
                myfile.close();
                
                cloud_pcl->width = cloud_pcl->points.size();
                cloud_pcl->height = 1;
                cloud_pcl->is_dense = true;
                const std::string pcd_path = time + ".pcd";
                pcl::io::savePCDFile(pcd_path, *cloud_pcl);
            }
            continue;
        }

        auto translation_ptr = robot_node->getField("translation");
        auto rotation_ptr = robot_node->getField("rotation");
        double tf_rotation[4], tf_translation[3]; 
        memcpy(tf_rotation, rotation_ptr->getSFRotation(), 4 * sizeof(tf_rotation[0]));
        memcpy(tf_translation, translation_ptr->getSFVec3f(), 3 * sizeof(tf_translation[0]));
        
        
        if(get_key == webots::Keyboard::UP)
        {
            double x =  0.02 * cos(tf_rotation[3]);
            double y =  0.02 * sin(tf_rotation[3]);
            tf_translation[0] += x;
            tf_translation[1] += y;
        }
        else if(get_key == webots::Keyboard::DOWN)
        {
            double x =  0.02 * cos(tf_rotation[3]);
            double y =  0.02 * sin(tf_rotation[3]);
            tf_translation[0] -= x;
            tf_translation[1] -= y;
        }
        else if(get_key == webots::Keyboard::LEFT)
        {
            tf_rotation[0] = 0;
            tf_rotation[1] = 0;
            tf_rotation[2] = 1;
            tf_rotation[3] += 1.0 / 360;
            std::ofstream myfile(log_file, std::ios::out|std::ios::app); 
            myfile << "tf_rotation = " << tf_rotation[0] << tf_rotation[1] << tf_rotation[2] << tf_rotation[3] <<std::endl;
            myfile.close();
        }
        else if(get_key == webots::Keyboard::RIGHT)
        {
            tf_rotation[0] = 0;
            tf_rotation[1] = 0;
            tf_rotation[2] = 1;
            tf_rotation[3] -= 1.0 / 360;
            std::ofstream myfile(log_file, std::ios::out|std::ios::app); 
            myfile << "tf_rotation = " << tf_rotation[0] << tf_rotation[1] << tf_rotation[2] << tf_rotation[3] <<std::endl;
            myfile.close();
        }
        else if(get_key == webots::Keyboard::CONTROL)
        {

        }
        rotation_ptr->setSFRotation(tf_rotation);
        translation_ptr->setSFVec3f(tf_translation);

    }
    camera->disable();
    lidar->disable();
    keyboard->disable();
    rangefinder->disable();
    return 0;
}
