#include "controllerthrd.h"

ControllerThrd::ControllerThrd(QObject *parent)
    : QThread{parent}
{
    _isManual=false;
    _steerSpeed=0;
    _steerYaw=0;
    _forkSpeed=0;
    _forkHeight=0;

}

void ControllerThrd::enableMotor()
{
    Supervisor *_super=Supervisor::getSupervisorInstance();
    Node *robot=_super->getFromDef("RobotNode");

    Motor *FL=_super->getMotor("FL");
    FL->setPosition(INFINITY);
    FL->setVelocity(0);

    Motor *FR=_super->getMotor("FR");
    FR->setPosition(INFINITY);
    FR->setVelocity(0);

    Motor *BL=_super->getMotor("BL");
    PositionSensor *BLPS=BL->getPositionSensor();
    BL->setPosition(INFINITY);
    BL->setVelocity(0);
    BLPS->enable(MillStep_5);

    Motor *BR=_super->getMotor("BR");
    PositionSensor *BRPS=BR->getPositionSensor();
    BR->setPosition(INFINITY);
    BR->setVelocity(0);
    BRPS->enable(MillStep_5);

    Motor *forkMotor=_super->getMotor("fork height motor");
    forkMotor->setPosition(INFINITY);
    forkMotor->setVelocity(0);

    PositionSensor *forkHeight=forkMotor->getPositionSensor();
    forkHeight->enable(MillStep_5);
}

void ControllerThrd::enableLidar3D()
{
    Supervisor *_super=Supervisor::getSupervisorInstance();
    Lidar *BP= _super->getLidar("BP");
    BP->enable(MillStep_5);
    BP->enablePointCloud();

    Node *mid360Node=_super->getFromDef("MID360");
    Lidar *mid360= _super->getLidar("mid360");
    mid360->enable(MillStep_5);
    mid360->enablePointCloud();

    Field *mid360_tf=mid360Node->getField("translation");
    const double *mid360_pose=mid360_tf->getSFVec3f();
    _mid360InitPose[0]=mid360_pose[0];
    _mid360InitPose[1]=mid360_pose[1];
    _mid360InitPose[2]=mid360_pose[2];
}

void ControllerThrd::setMid360Height()
{
    Supervisor *_super=Supervisor::getSupervisorInstance();
    Motor *forkMotor=_super->getMotor("fork height motor");
    PositionSensor *forkHeight=forkMotor->getPositionSensor();
    _forkHeight=forkHeight->getValue();

    Node *mid360Node=_super->getFromDef("MID360");
    ///感知激光随动
    double deltaHeightBetweenForkAndMid360=forkHeight->getValue()-0.275;
    if(deltaHeightBetweenForkAndMid360>0)
    {
        Field *mid360_tf=mid360Node->getField("translation");
        double pose[3]={_mid360InitPose[0],_mid360InitPose[1],_mid360InitPose[2]+deltaHeightBetweenForkAndMid360};
        mid360_tf->setSFVec3f(pose);
    }else
    {
        Field *mid360_tf=mid360Node->getField("translation");
        mid360_tf->setSFVec3f(_mid360InitPose);
    }
}

void ControllerThrd::enableLidar2D()
{
    Supervisor *_super=Supervisor::getSupervisorInstance();
    Lidar *lidar2D= _super->getLidar("lidar2D");
    lidar2D->enable(MillStep_5);
    lidar2D->enablePointCloud();
}

void ControllerThrd::enableIMU()
{
    Supervisor *_super=Supervisor::getSupervisorInstance();
    InertialUnit *inertialUnit=_super->getInertialUnit("inertial unit");
    inertialUnit->enable(MillStep_5);

    Gyro *gyro=_super->getGyro("gyro");
    gyro->enable(MillStep_5);

    Accelerometer* accelerometer=_super->getAccelerometer("accelerometer");
    accelerometer->enable(MillStep_5);
}

void ControllerThrd::publishIMU()
{
    Supervisor *_super=Supervisor::getSupervisorInstance();
    InertialUnit *inertialUnit=_super->getInertialUnit("inertial unit");
    ///get yaw
    const double *data=inertialUnit->getRollPitchYaw();///r p y
    //qDebug()<<data[0]*180/PI<<data[1]*180/PI<<data[2]*180/PI;

    foxglove::ImuInFrame imu_in_frame;
    imu_in_frame.set_frame_id("imu_link");
    struct timeval tv;
    gettimeofday(&tv, NULL);
    google::protobuf::Timestamp *timestamp=imu_in_frame.mutable_timestamp();
    timestamp->set_seconds(tv.tv_sec);
    timestamp->set_nanos(tv.tv_usec * 1000);

    foxglove::Imu *imu=imu_in_frame.mutable_imu();

    const double *qua=inertialUnit->getQuaternion();///x y z w
    foxglove::Quaternion* q=imu->mutable_orientation();
    q->set_x(qua[0]);
    q->set_y(qua[1]);
    q->set_z(qua[2]);
    q->set_w(qua[3]);

    Gyro *gyro=_super->getGyro("gyro");
    const double *angular_velocity_data=gyro->getValues();
    foxglove::Vector3 *angular_velocity=imu->mutable_angular_velocity();
    angular_velocity->set_x(angular_velocity_data[0]);
    angular_velocity->set_y(angular_velocity_data[1]);
    angular_velocity->set_z(angular_velocity_data[2]);


    Accelerometer* accelerometer=_super->getAccelerometer("accelerometer");
    const double *accelerometer_data=accelerometer->getValues();
    foxglove::Vector3 *accelerometer_velocity=imu->mutable_linear_acceleration();
    accelerometer_velocity->set_x(accelerometer_data[0]);
    accelerometer_velocity->set_y(accelerometer_data[1]);
    accelerometer_velocity->set_z(accelerometer_data[2]);

    _imu_publisher.Send(imu_in_frame);
    bridge.publishRosImu(&imu_in_frame);
}

void ControllerThrd::publishPointCLoud()
{
    Supervisor *_super=Supervisor::getSupervisorInstance();
    Lidar *lidar2D= _super->getLidar("lidar2D");
    bridge.publishRosLaserScan(lidar2D);
}

void ControllerThrd::initPbulisher()
{
    eCAL::Initialize(0, 0, "general_controller");
    _imu_publisher.Create("/imu");

    bridge.initRosPublisher();
}

void ControllerThrd::run()
{
    Supervisor *_super= new Supervisor();
    initPbulisher();
    enableMotor();
    enableLidar3D();
    enableLidar2D();
    enableIMU();


    QElapsedTimer  t;
    t.start();
    int count=1;
    while (_super->step(2) != -1)
    {
        qDebug()<<"start:"<<t.elapsed();
        setSteerWheelYaw(_steerYaw);
        qDebug()<<"setSteerWheelYaw:"<<t.elapsed();
        setRobotSpeed(_steerYaw,_steerSpeed);
        qDebug()<<"setRobotSpeed:"<<t.elapsed();
        setForkSpeed(_forkSpeed);

        qDebug()<<"setForkSpeed:"<<t.elapsed();
        setMid360Height();

        qDebug()<<"setMid360Height:"<<t.elapsed();
        publishIMU();
        qDebug()<<"publishIMU:"<<t.elapsed();
        // if(count%5==0)
        // {

            publishPointCLoud();
        qDebug()<<"publishPointCLoud:"<<t.elapsed();
       // }
        t.restart();

        count++;
    }
}

