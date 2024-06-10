#ifndef __NRLS__H__
#define __NRLS__H__

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include "sim_data_flow/point_cloud.pb.h"
#include "logvn/logvn.h"

#define M_PI 3.14

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
    // float min_Z1{36000};
    float min_Z2{3500};
};

class NRLS {
   private:
    std::vector<struct FBContain> fb_list_;

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
    int load(const char *path, const struct LidarInfo &input) {
        int ret = 0;
        std::vector<std::string> csv_data_;
        do {
            // TODO: is there need try{}catch(...)?
            if (this->readCSV(path, csv_data_) != 0) {
                LOG_ERROR("unalble to load csv --> %s", path);
                ret = -1;
                break;
            }

            // 计算查找表
            for (auto &t : csv_data_) {
                int index;
                FBContain tmp;
                std::sscanf(t.c_str(), "%d,%f,%f", &index, &tmp.horizon_angle,
                            &tmp.vertical_angle);

                // 计算稠密点云index
                tmp.horizon_angle = tmp.horizon_angle * 100.f;
                tmp.vertical_angle = tmp.vertical_angle * 100.f;

                float sim_h_res = input.fieldOfView * 180 / M_PI * 100 /
                                  input.horizontalResolution;
                float sim_v_res = input.verticalFieldOfView * 180 / M_PI * 100 /
                                  input.numberOfLayers;
                float sim_h_res_t = 1. / sim_h_res;
                float sim_v_res_t = 1. / sim_v_res;

                // 行号，列号
                // TODO: what is minZ2? check
                tmp.layer_count =
                    (tmp.vertical_angle - input.min_Z2) * sim_v_res_t;
                tmp.pc_idx = tmp.horizon_angle * sim_h_res_t;

                fb_list_.push_back(tmp);
            }
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
                    sim_data_flow::WBPointCloud &point_cloud) {
        int layers = point_cloud.size_of_layer();
        int npl = point_cloud.size_of_point_cloud();

        point_cloud.clear_point_cloud();

        // 遍历查找表
        for (const auto &unit : fb_list_) {
            if (unit.layer_count < 0 || unit.pc_idx < 0)
                continue;
            if (unit.layer_count >= layers || unit.pc_idx >= npl)
                continue;

            //
            const LidarPoint *cur_ptr =
                (source + (unit.layer_count * layers + unit.pc_idx));

            if (checkPointIsFine(cur_ptr)) {
                sim_data_flow::LidarPoint *point =
                    point_cloud.add_point_cloud();
                point->set_x(cur_ptr->x);
                point->set_y(cur_ptr->y);
                point->set_z(cur_ptr->z);
                point->set_time(cur_ptr->time);
                point->set_layer_id(cur_ptr->layer_id);
            }
        }
        point_cloud.set_size_of_each_layer(-1);
        point_cloud.set_size_of_layer(-1);
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
};
}  // namespace VNSim

#endif