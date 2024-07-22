#include <vector>
#include "geometry.h"

namespace VNSim {

double Deg2Rad(double x) {
    return double(x * PI / 180);
}
double Rad2Deg(double x) {
    return double(x * 180 / PI);
}

////[0,2π]
double NormalizeTo2PI(double theta) {
    const double TWO_PI = 2 * M_PI;     
    theta = fmod(theta, TWO_PI);

    if (theta < 0)
        theta += TWO_PI;

    return theta;
}

/// [−π:π]
double NormalizeToPI(double theta) {
    const double PI = M_PI; // 使用 <cmath> 中的 M_PI 常量
    const double TWO_PI = 2 * PI;

    // 使用 fmod 将 theta 归一化到 [-2*PI, 2*PI) 范围
    theta = fmod(theta, TWO_PI);
    
    // 将 theta 调整到 [-PI, PI] 范围
    if (theta < -PI) {
        theta += TWO_PI;
    } else if (theta > PI) {
        theta -= TWO_PI;
    }

    return theta;
}

double NormalizeAngleTo360(double angle) {
    const double TWO_PI = 360.0; // 2π 弧度转换为度数
    angle = fmod(angle, TWO_PI);

    if (angle < 0)
        angle += TWO_PI; // 如果角度是负数，将其转换为正数

    return angle;
}

double NormalizeAngleTo180(double angle) {
    const double TWO_PI = 360.0; // 2π 弧度转换为度数
    const double PI = 180.0;     // π 弧度转换为度数
    
    // 首先将角度归一化到 [0, 360)
    angle = fmod(angle, TWO_PI);

    // 如果角度是负数，加上 360 度使其变为正数
    if (angle < 0)
        angle += TWO_PI;

    // 将角度调整到 [-180, 180] 范围内
    if (angle > PI) {
        angle -= TWO_PI; // 如果大于 180 度，减去 360 度
    }

    return angle;
}

/// Eigen库中欧拉角范围
/// 0:[−π:π]右滚为正;1:[−π:π]抬头为负;2:[−π:π]左为正

//四元数-->欧拉角
Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond q) {
    Eigen::AngleAxisd a(q);
    Eigen::Vector3d e = a.matrix().eulerAngles(2, 1, 0);  /// yaw pitch roll
    return Eigen::Vector3d(e[2], e[1], e[0]);
}

//欧拉角-->四元数
///用eigen库的范围要求描述角度
Eigen::Quaterniond Euler2Quaternion(Eigen::Vector3d euler) {
    // yaw pitch roll
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
}

inline Eigen::Vector3d rotVecToEuler(const double *vec) {
    Eigen::AngleAxisd tmp_angleaxis(vec[3],
                                    Eigen::Vector3d(vec[0], vec[1], vec[2]));
    Eigen::Vector3d ret = tmp_angleaxis.matrix().eulerAngles(2, 1, 0);
    return ret;
}

inline Eigen::Vector3d rotVecToEuler(const Eigen::AngleAxisd &vec) {
    Eigen::Vector3d ret = vec.matrix().eulerAngles(2, 1, 0);
    return ret;
}

inline Eigen::Quaterniond rotVecToQua(const double *vec) {
    double x = vec[0];
    double y = vec[1];
    double z = vec[2];
    double unit = vec[3];

    double l = x * x + y * y + z * z;
    double mW, mX, mY, mZ;
    auto unit_t = unit;
    if (l > 0.0) {
        unit_t *= 0.5;
        mW = cos(unit_t);
        l = sin(unit_t) / sqrt(l);
        mX = x * l;
        mY = y * l;
        mZ = z * l;
    } else {
        mW = 1.0;
        mX = 0.0;
        mY = 0.0;
        mZ = 0.0;
    }
    return Eigen::Quaterniond(mW, mX, mY, mZ);
}

inline Eigen::AngleAxisd eulerToRotVec(double roll, double pitch, double yaw) {
    using Eigen::AngleAxisd;
    using Eigen::Vector3d;
    AngleAxisd rollAngle(AngleAxisd(roll, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(pitch, Vector3d::UnitY()));
    AngleAxisd yawlAngle(AngleAxisd(yaw, Vector3d::UnitZ()));
    AngleAxisd ret;
    ret = yawlAngle * pitchAngle * rollAngle;
    return ret;
}
///
/// \brief eulerToRotVec
/// \param euler_angle : order by roll, pitch, yawl
/// \return
///
inline Eigen::AngleAxisd eulerToRotVec(const Eigen::Vector3d &euler_angle) {
    using Eigen::AngleAxisd;
    using Eigen::Vector3d;

    AngleAxisd rollAngle(AngleAxisd(euler_angle(0), Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(euler_angle(1), Vector3d::UnitY()));
    AngleAxisd yawlAngle(AngleAxisd(euler_angle(2), Vector3d::UnitZ()));
    AngleAxisd ret;
    ret = yawlAngle * pitchAngle * rollAngle;
    return ret;
}

///
/// \brief eulerToWbtRot
/// \param roll
/// \param pitch
/// \param yaw
/// \return size of vector is 4
///
inline std::vector<double> eulerToWbtRot(double roll, double pitch,
                                         double yaw) {
    using Eigen::AngleAxisd;
    using Eigen::Vector3d;
    AngleAxisd rollAngle(AngleAxisd(roll, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(pitch, Vector3d::UnitY()));
    AngleAxisd yawlAngle(AngleAxisd(yaw, Vector3d::UnitZ()));
    AngleAxisd angle_axis;
    angle_axis = yawlAngle * pitchAngle * rollAngle;

    std::vector<double> ret = {angle_axis.axis()(0), angle_axis.axis()(1),
                               angle_axis.axis()(2), angle_axis.angle()};
    return ret;
}

inline std::vector<double> eulerToWbtRot(const Eigen::Vector3d &euler_angle) {
    using Eigen::AngleAxisd;
    using Eigen::Vector3d;

    AngleAxisd rollAngle(AngleAxisd(euler_angle(2), Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(euler_angle(1), Vector3d::UnitY()));
    AngleAxisd yawlAngle(AngleAxisd(euler_angle(0), Vector3d::UnitZ()));
    AngleAxisd angle_axis;
    angle_axis = yawlAngle * pitchAngle * rollAngle;

    std::vector<double> ret = {angle_axis.axis()(0), angle_axis.axis()(1),
                               angle_axis.axis()(2), angle_axis.angle()};
    return ret;
}

inline Eigen::Quaterniond eulerToQua(double roll, double pitch, double yaw) {
    using Eigen::AngleAxisd;
    using Eigen::Quaterniond;
    using Eigen::Vector3d;

    AngleAxisd rollAngle(AngleAxisd(roll, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(pitch, Vector3d::UnitY()));
    AngleAxisd yawlAngle(AngleAxisd(yaw, Vector3d::UnitZ()));
    Quaterniond ret;
    ret = yawlAngle * pitchAngle * rollAngle;
    return ret;
}

inline Eigen::Quaterniond eulerToQua(const Eigen::Vector3d &euler_angle) {
    using Eigen::AngleAxisd;
    using Eigen::Quaterniond;
    using Eigen::Vector3d;
    AngleAxisd rollAngle(AngleAxisd(euler_angle(2), Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(euler_angle(1), Vector3d::UnitY()));
    AngleAxisd yawlAngle(AngleAxisd(euler_angle(0), Vector3d::UnitZ()));
    Quaterniond ret;
    ret = yawlAngle * pitchAngle * rollAngle;
    return ret;
}

Eigen::Matrix4d createTransformMatrix(const double rotation[4],
                                      const double translation[3]) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    // 创建一个轴角表示
    Eigen::AngleAxisd angleAxis(
        rotation[3], Eigen::Vector3d(rotation[0], rotation[1], rotation[2]));

    // 将轴角转换为四元数
    Eigen::Quaterniond q = Eigen::Quaterniond(angleAxis);

    // 通过四元数创建旋转矩阵
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();

    // 将旋转矩阵和平移向量填充到4x4变换矩阵中
    transform.block<3, 3>(0, 0) = rotationMatrix;
    transform.block<3, 1>(0, 3) =
        Eigen::Vector3d(translation[0], translation[1], translation[2]);

    return transform;
}

Eigen::Matrix4d poseToMatrix4d(Eigen::Quaterniond rotation,
                               Eigen::Vector3d translation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    // 通过四元数创建旋转矩阵
    Eigen::Matrix3d rotationMatrix = rotation.normalized().toRotationMatrix();

    // 将旋转矩阵和平移向量填充到4x4变换矩阵中
    transform.block<3, 3>(0, 0) = rotationMatrix;
    transform.block<3, 1>(0, 3) = translation;

    return transform;
}

double dotProduct(const Eigen::Vector4d &a, const Eigen::Vector4d &b) {
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}
}  // namespace VNSim
