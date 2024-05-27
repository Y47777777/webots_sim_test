#include "utils.hpp"
// Math Utils

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