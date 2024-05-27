#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

// Math Utils

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

#endif
