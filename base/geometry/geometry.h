#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace VNSim {

const double PI = 3.1415926;

double Deg2Rad(double x);
double Rad2Deg(double x);

////[0,2π]
double Normalize2(double theta);

/// [−π:π]
double Normalize(double theta);

/// Eigen库中欧拉角范围
/// 0:[−π:π]右滚为正;1:[−π:π]抬头为负;2:[−π:π]左为正

//四元数-->欧拉角
Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond q);
//欧拉角-->四元数
///用eigen库的范围要求描述角度
Eigen::Quaterniond Euler2Quaternion(Eigen::Vector3d euler);

inline Eigen::Vector3d rotVecToEuler(const double *vec);

inline Eigen::Vector3d rotVecToEuler(const Eigen::AngleAxisd &vec);

inline Eigen::Quaterniond rotVecToQua(const double *vec);

inline Eigen::AngleAxisd eulerToRotVec(double roll, double pitch, double yaw);

///
/// \brief eulerToRotVec
/// \param euler_angle : order by roll, pitch, yawl
/// \return
///
inline Eigen::AngleAxisd eulerToRotVec(const Eigen::Vector3d &euler_angle);

///
/// \brief eulerToWbtRot
/// \param roll
/// \param pitch
/// \param yaw
/// \return size of vector is 4
///
inline std::vector<double> eulerToWbtRot(double roll, double pitch, double yaw);

inline std::vector<double> eulerToWbtRot(const Eigen::Vector3d &euler_angle);

inline Eigen::Quaterniond eulerToQua(double roll, double pitch, double yaw);

inline Eigen::Quaterniond eulerToQua(const Eigen::Vector3d &euler_angle);

}  // namespace VNSim
