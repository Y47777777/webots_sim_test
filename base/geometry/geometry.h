#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <iostream>
#include <random>

namespace VNSim {

const double PI = 3.1415926;

double Deg2Rad(double x);
double Rad2Deg(double x);

////归一角度
double NormalizeTo2PI(double theta);
double NormalizeToPI(double theta);
double NormalizeAngleTo360(double angle);
double NormalizeAngleTo180(double angle);

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

Eigen::Matrix4d createTransformMatrix(const double rotation[4],
                                      const double translation[3]);

Eigen::Matrix4d poseToMatrix4d(Eigen::Quaterniond rotation,
                               Eigen::Vector3d translation);

double dotProduct(const Eigen::Vector4d &a, const Eigen::Vector4d &b);

class RandomGenerator {
   private:
    std::mt19937 gen;
    std::uniform_real_distribution<> distr;

   public:
    // 构造函数，初始化随机数生成器和分布
    RandomGenerator(double range = 0.0)
        : gen(std::random_device{}()),  // 使用初始化列表初始化随机数生成器
          distr(0.0, range) {}  // 初始化分布器

    // 生成随机数的方法
    double generate() { return distr(gen); }
};

}  // namespace VNSim
