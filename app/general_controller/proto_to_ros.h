#ifndef PROTO_TO_ROS_H
#define PROTO_TO_ROS_H

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"///horizontal_laser_2d
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "ecal/ecal.h"
#include "ecal/msg/protobuf/publisher.h"
#include "foxglove-vn/ImuInFrame.pb.h"
#include "ros/publisher.h"
#include "ros/common.h"
#include <ros/ros.h>
#include <webots/Lidar.hpp>
#include <QDebug>
#include <QtCore/QElapsedTimer>
using namespace webots;

class RosBridge
{
public:
    RosBridge()
    {
    }

    void initRosPublisher()
    {
        int argc=0;
        char **argv=NULL;
        ros::init(argc, argv, "general_controller");
        ros::NodeHandle nh;
        _ros_imu_publisher = nh.advertise<sensor_msgs::Imu> ("imu", 1000);
        _ros_laserScan_publisher = nh.advertise<sensor_msgs::LaserScan> ("scan", 1000);
    }

    void publishRosImu(foxglove::ImuInFrame *imu_in_frame)
    {
        sensor_msgs::Imu ros_imu;
        ros_imu.header.frame_id=imu_in_frame->frame_id();
        ros_imu.header.stamp=ros::Time(imu_in_frame->timestamp().seconds(),imu_in_frame->timestamp().nanos());
        ros_imu.orientation.x=imu_in_frame->imu().orientation().x();
        ros_imu.orientation.y=imu_in_frame->imu().orientation().y();
        ros_imu.orientation.z=imu_in_frame->imu().orientation().z();
        ros_imu.orientation.w=imu_in_frame->imu().orientation().w();

        ros_imu.angular_velocity.x=imu_in_frame->imu().angular_velocity().x();
        ros_imu.angular_velocity.y=imu_in_frame->imu().angular_velocity().y();
        ros_imu.angular_velocity.z=imu_in_frame->imu().angular_velocity().z();

        ros_imu.linear_acceleration.x=imu_in_frame->imu().linear_acceleration().x();
        ros_imu.linear_acceleration.y=imu_in_frame->imu().linear_acceleration().y();
        ros_imu.linear_acceleration.z=imu_in_frame->imu().linear_acceleration().z();

        _ros_imu_publisher.publish(ros_imu);
    }

    void publishRosLaserScan(Lidar *lidar2D)
    {
        int horizontalResolution=360;
        int frequency=20;

        sensor_msgs::LaserScan ros_laserScan;
        ///ros扫描是逆时针,从正前方(x方向)开始扫描的.
        ///webots是顺时针从x轴左侧fieldOfView/2处开始扫描

        ros_laserScan.header.frame_id="horizontal_laser_link";
        struct timeval tv;
        gettimeofday(&tv, NULL);
        ros_laserScan.header.stamp=ros::Time(tv.tv_sec,tv.tv_usec * 1000);

        qDebug()<<ros_laserScan.header.stamp.sec<<ros_laserScan.header.stamp.nsec;

        ros_laserScan.angle_min=0;
        ros_laserScan.angle_max=2*3.14;
        ros_laserScan.angle_increment=(ros_laserScan.angle_max-ros_laserScan.angle_min)/horizontalResolution;

        ros_laserScan.time_increment = (1 / frequency) / (horizontalResolution);

        ros_laserScan.range_min=lidar2D->getMinRange();
        ros_laserScan.range_max=lidar2D->getMaxRange();

        ros_laserScan.ranges.resize(horizontalResolution);
        ros_laserScan.intensities.resize(horizontalResolution);

        const LidarPoint *pc=lidar2D->getPointCloud();
        for(int i=0;i<lidar2D->getNumberOfPoints();i++)
        {
            if(pc[i].x!=INFINITY&&pc[i].y!=INFINITY&&pc[i].z!=INFINITY)
            {
                //qDebug()<<pc[i].x<<pc[i].y<<pc[i].z;

                if(i<180)
                {
                    ros_laserScan.ranges[179-i]=sqrt(pc[i].x*pc[i].x+pc[i].y*pc[i].y);
                    ros_laserScan.intensities[179-i]=100;
                }else
                {
                    ros_laserScan.ranges[359-(i-180)]=sqrt(pc[i].x*pc[i].x+pc[i].y*pc[i].y);
                    ros_laserScan.intensities[359-(i-180)]=100;
                }
            }

        }
        _ros_laserScan_publisher.publish(ros_laserScan);

    }

private:
    ros::Publisher _ros_imu_publisher;
    ros::Publisher _ros_laserScan_publisher;

};



#endif // PROTO_TO_ROS_H
