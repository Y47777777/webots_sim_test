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
double Normalize2(double theta) {
    if (theta < 2 * PI && theta >= 0.0)
        return theta;
    double multiplier = floor(theta / (2 * PI));
    theta = theta - multiplier * 2 * PI;
    if (theta < 0.0)
        theta += 2 * PI;
    if (theta > 2 * PI - 0.01)
        theta = 0.0;
    return theta;
}

/// [−π:π]
double Normalize(double theta) {
    double t = Normalize2(theta);
    if (t > PI)
        return t - 2 * PI;
    else
        return t;
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

}  // namespace VNSim
