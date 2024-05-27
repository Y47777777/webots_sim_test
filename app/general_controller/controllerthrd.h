#ifndef CONTROLLERTHRD_H
#define CONTROLLERTHRD_H

#include <QObject>
#include <QThread>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include <webots/Lidar.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Camera.hpp>
#include <webots/VacuumGripper.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Accelerometer.hpp>
#include <QString>
#include <QVector>
#include <QDebug>


#include "proto_to_ros.h"


using namespace std;
using namespace webots;

class ControllerThrd : public QThread
{
public:
    explicit ControllerThrd(QObject *parent = nullptr);


    void run();

    void setMenual(bool isManual)
    {
        _isManual=isManual;
    }
    void speedChanged(double steerSpeed,double steerYaw,double forkSpeed)
    {
        if(_isManual)
        {
            _steerSpeed=steerSpeed;
            _steerYaw=steerYaw;
            _forkSpeed=forkSpeed;
        }
    }

    void getStatus(double &steerSpeed,double &steerYaw,double &forkSpeed,double &forkHeight)
    {
        steerSpeed=_steerSpeed;
        steerYaw=_steerYaw;
        forkSpeed=_forkSpeed;
        forkHeight=_forkHeight;
    }

private:
    bool _isManual;
    double _steerSpeed;
    double _steerYaw;
    double _forkSpeed;
    double _forkHeight;
    double _mid360InitPose[3]={0,0,0};

    const int MillStep_5=5;
    const int MillStep_10=10;
    const int MillStep_50=50;
    const int MillStep_100=100;

    RosBridge bridge;
    eCAL::protobuf::CPublisher<foxglove::ImuInFrame> _imu_publisher;


private:
    void initPbulisher();
    void enableMotor();
    void enableLidar3D();
    void setMid360Height();
    void enableLidar2D();
    void enableIMU();

    void publishIMU();
    void publishPointCLoud();


    const double PI=3.1415926;


    double Deg2Rad( double x )
    {
        return double(x*PI/180);
    }
    double Rad2Deg( double x )
    {
        return double(x*180/PI);
    }

    ////[0,2π]
    double Normalize2(double theta)
    {
        if(theta<2*PI&&theta>=0.0)
            return theta;
        double multiplier = floor(theta / (2*PI));
        theta = theta - multiplier*2*PI;
        if(theta<0.0)
            theta+=2*PI;
        if(theta>2*PI-0.01)
            theta=0.0;
        return theta;
    }

    /// [−π:π]
    double Normalize(double theta)
    {
        double t=Normalize2(theta);
        if (t > PI)
            return t-2*PI;
        else
            return t;
    }

    ///Eigen库中欧拉角范围
    /// 0:[−π:π]右滚为正;1:[−π:π]抬头为负;2:[−π:π]左为正


    //四元数-->欧拉角
    Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond q)
    {
        Eigen::AngleAxisd a(q);
        Eigen::Vector3d e=a.matrix().eulerAngles(2,1,0);///yaw pitch roll
        return Eigen::Vector3d(e[2],e[1],e[0]);
    }

    //欧拉角-->四元数
    ///用eigen库的范围要求描述角度
    Eigen::Quaterniond Euler2Quaternion(Eigen::Vector3d euler)
    {
        //yaw pitch roll
        return Eigen::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
    }

    ////[-pi/2,pi/2]  左为正
    void setSteerWheelYaw(double yaw)
    {
        if(yaw>PI/2)
            yaw=PI/2;
        else if(yaw<-PI/2)
            yaw=-PI/2;
        Eigen::Vector3d origin_e(-PI/2,0,0);
        Eigen::Vector3d new_e(-PI/2,0,0+yaw);

        Eigen::Quaterniond q;
        q=Euler2Quaternion(new_e);///按zyx轴的顺序旋转

        Eigen::AngleAxisd a(q);
        double new_aa[4]={a.axis().x(),a.axis().y(),a.axis().z(),a.angle()};

        Supervisor *_super=Supervisor::getSupervisorInstance();
        Node *steerWheel=_super->getFromDef("SteerWheel");
        Field *steerWheelField=steerWheel->getField("rotation");
        steerWheelField->setSFRotation(new_aa);
    }

    void setRobotSpeed(double yaw,double v)
    {
        Supervisor *_super=Supervisor::getSupervisorInstance();
        Motor *FL=_super->getMotor("FL");
        Motor *FR=_super->getMotor("FR");
        Motor *BL=_super->getMotor("BL");
        Motor *BR=_super->getMotor("BR");


        double steerWheelX=1.02;
        double drivenWheelY=0.473;

        double driven_wheel_l_v;
        double driven_wheel_r_v;

        double abs_yaw=std::fabs(yaw);

        if(abs_yaw<0.1)
        {
            driven_wheel_l_v=v;
            driven_wheel_r_v=v;
        }
        else if(abs_yaw>PI/2-0.1)
        {
            driven_wheel_l_v=-v*drivenWheelY/steerWheelX;
            driven_wheel_r_v=v*drivenWheelY/steerWheelX;
        }
        else
        {
            double d= steerWheelX/std::sin(abs_yaw);
            double dl=steerWheelX/std::tan(abs_yaw)-drivenWheelY;
            double dr=dl+2*drivenWheelY;
            //qDebug()<<"d:"<<d<<"dl:"<<dl<<"dr:"<<dr;
            driven_wheel_l_v=v*dl/d;
            driven_wheel_r_v=v*dr/d;
        }

        if(yaw<0)
        {
            std::swap(driven_wheel_l_v,driven_wheel_r_v);
        }

        FL->setVelocity(v);
        FR->setVelocity(v);
        BL->setVelocity(driven_wheel_l_v);
        BR->setVelocity(driven_wheel_r_v);

        //qDebug()<<"v:"<<v<<"vl:"<<driven_wheel_l_v<<"vr:"<<driven_wheel_r_v;
    }

    void setForkSpeed(double v)
    {
        Supervisor *_super=Supervisor::getSupervisorInstance();
        Motor *forkMotor=_super->getMotor("fork height motor");
        forkMotor->setVelocity(v);
    }


};

#endif // CONTROLLERTHRD_H
