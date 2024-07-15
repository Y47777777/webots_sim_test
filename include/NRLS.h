#ifndef __NRLS__H__
#define __NRLS__H__

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include "sim_data_flow/point_cloud.pb.h"
#include "lidar_simulation/high_reflector.h"
#include "logvn/logvn.h"

#define M_PI 3.14
#define MID360_ONCE_CLOUD_SIZE 20722
#define HAP_ONCE_CLOUD_SIZE 45200
namespace VNSim {

struct FBContain {
    int pc_idx;
    int layer_count;
    double horizon_angle;
    double vertical_angle;
};

struct LidarInfo {
    // TODO: is this struct necessary?
    //      if it`s necessary, please comply with the specifiance
    int scan_frequency_{100};
    int horizontalResolution;    // read
    double fieldOfView;          // read
    double verticalFieldOfView;  // read
    int numberOfLayers;          // read
};

class NRLS {
   private:
    std::vector<struct FBContain> fb_list_;
    int last_point_{-1};
    std::vector<struct FBContain>::iterator list_iter_;

   public:
    NRLS() {}
    ~NRLS() {}

    /**
     * @brief 读取非重复线扫模拟表
     *
     * @param[in] path   文件路径
     * @param[in] input  雷达参数
     * @return int
     */
    int load(const char *path, const struct LidarInfo &input,
             size_t result_cloud_size) {
        int ret = 0;
        std::vector<std::string> csv_data_;
        result_cloud_size_ = result_cloud_size;
        do {
            // TODO: is there need try{}catch(...)?
            if (this->readCSV(path, csv_data_) != 0) {
                LOG_ERROR("unalble to load csv --> %s", path);
                ret = -1;
                break;
            }

            bool first_flag = true;

            // 转存
            for (auto &t : csv_data_) {
                // 去除第一行
                if (first_flag) {
                    first_flag = !first_flag;
                    continue;
                }

                double index;
                FBContain tmp;
                std::sscanf(t.c_str(), "%lf,%lf,%lf", &index,
                            &tmp.horizon_angle, &tmp.vertical_angle);

                tmp.horizon_angle  = NormalizeAngleTo180(tmp.horizon_angle);
                tmp.vertical_angle = NormalizeAngleTo180(tmp.vertical_angle);

                fb_list_.push_back(tmp);
            }

            // 计算分辨率
            double sim_ver_res = double(input.numberOfLayers)       / Rad2Deg(input.verticalFieldOfView);
            double sim_hor_res = double(input.horizontalResolution) / Rad2Deg(input.fieldOfView);

            LOG_INFO("ver_res:%f, sim_hor_res: %.f", sim_ver_res, sim_hor_res);

            // webots point start point layer angle
            double veritcal_origin   = 90.0 - (Rad2Deg(input.verticalFieldOfView) / 2);
            double horizontal_origin = -Rad2Deg(input.fieldOfView) / 2;

            LOG_INFO("origin veritcal:%f, horizontal: %.f", veritcal_origin, horizontal_origin);

            int min_layer = MAXFLOAT;
            int max_layer = 0;

            int min_index = MAXFLOAT;
            int max_index = 0;
            // 建立查找表
            for (auto &t : fb_list_) {
                t.layer_count = std::round(
                    (t.vertical_angle - veritcal_origin) * sim_ver_res);
                t.pc_idx = std::round((t.horizon_angle - horizontal_origin) *
                                      sim_hor_res);

                // 记录最大最小值
                if (min_layer > t.layer_count) {
                    min_layer = t.layer_count;
                }
                if (max_layer < t.layer_count) {
                    max_layer = t.layer_count;
                }
                if (min_index > t.pc_idx) {
                    min_index = t.pc_idx;
                }
                if (max_index < t.pc_idx) {
                    max_index = t.pc_idx;
                }

                // 防止溢出
                if (t.pc_idx < 0) {
                    t.pc_idx = 0;
                }

                if (t.pc_idx >= input.horizontalResolution) {
                    t.pc_idx = input.horizontalResolution - 1;
                }

                if (t.layer_count < 0) {
                    t.layer_count = 0;
                }

                if (t.layer_count >= input.numberOfLayers) {
                    t.layer_count = input.numberOfLayers - 1;
                }
            }

            list_iter_ = fb_list_.begin();
            LOG_INFO("fb_list_  size %d", fb_list_.size());
            LOG_INFO("min layer %d,max %d", min_layer, max_layer);
            LOG_INFO("min index %d,max %d", min_index, max_index);
            LOG_INFO("horizontal %d, vertial %d ", input.horizontalResolution, input.numberOfLayers);
            
        } while (0);
        return ret;
    }

    /**
     * @brief 模拟非重复线扫
     *
     * @param[in]  source           源点云
     * @param[out] point_cloud      输出点云
     *                              点云行数，列数放在result的size_of_*中
     */
    void simulation(const LidarPoint *source,
                    sim_data_flow::WBPointCloud &point_cloud,
                    std::string lidar_name) {
        int layers = point_cloud.size_of_layer();
        int size_of_each_layer = point_cloud.size_of_each_layer();

        // 每次只取 指定 个点
        // 遍历查找表
        for (int i = 0; i < result_cloud_size_; i++, list_iter_++) {
            if (list_iter_ == fb_list_.end()) {
                list_iter_ = fb_list_.begin();
            }

            if (list_iter_->layer_count < 0 || list_iter_->pc_idx < 0) {
                // LOG_INFO("layers %d, list_iter_->layer_count, %d",        layers, list_iter_->layer_count);
                // LOG_INFO("size_of_each_layer %d, list_iter_->pc_idx, %d", size_of_each_layer, list_iter_->pc_idx);
                continue;
            }

            if (list_iter_->layer_count >= layers ||
                list_iter_->pc_idx >= size_of_each_layer) {
                // LOG_INFO("layers %d, list_iter_->layer_count, %d",        layers, list_iter_->layer_count);
                // LOG_INFO("size_of_each_layer %d, list_iter_->pc_idx, %d", size_of_each_layer, list_iter_->pc_idx);
                continue;
            }

            // 取点
            const LidarPoint *cur_ptr =
                (source + (list_iter_->layer_count * size_of_each_layer) +
                 list_iter_->pc_idx);

            if (checkPointIsFine(cur_ptr)) {
                sim_data_flow::LidarPoint *point =
                    point_cloud.add_point_cloud();
                point->set_x(cur_ptr->x);
                point->set_y(cur_ptr->y);
                point->set_z(cur_ptr->z);
                point->set_time(cur_ptr->time);
                point->set_layer_id(cur_ptr->layer_id);
                point->set_intensity(150);
            }
        }
    }

   private:
    int readCSV(const char *path, std::vector<std::string> &csv_data) {
        std::ifstream file(path);
        if (!file.is_open())
            return -1;
        char tmp[256];
        while (file.getline(tmp, 256)) { csv_data.push_back(tmp); }
        file.close();
        return 0;
    }

    bool checkPointIsFine(const LidarPoint *ptr) {
        return (std::abs(ptr->x) != INFINITY && std::abs(ptr->y) != INFINITY &&
                std::abs(ptr->z) != INFINITY);
    }

    // 每帧点云的数量
    size_t result_cloud_size_ = MID360_ONCE_CLOUD_SIZE;
};
}  // namespace VNSim

#endif