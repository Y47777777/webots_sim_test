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
    int load(const char *path, const struct LidarInfo &input) {
        int ret = 0;
        std::vector<std::string> csv_data_;
        do {
            if (this->readCSV(path, csv_data_) != 0) {
                LOG_ERROR("unalble to load csv --> %s", path);
                ret = -1;
                break;
            }
            for (auto &t : csv_data_) {
                int index;
                FBContain tmp;
                std::sscanf(t.c_str(), "%d,%f,%f", &index, &tmp.horizon_angle,
                            &tmp.vertical_angle);
                tmp.horizon_angle = tmp.horizon_angle * 100.f;
                tmp.vertical_angle = tmp.vertical_angle * 100.f;
                float sim_h_res = input.fieldOfView * 180 / M_PI * 100 /
                                  input.horizontalResolution;
                float sim_v_res = input.verticalFieldOfView * 180 / M_PI * 100 /
                                  input.numberOfLayers;
                float sim_h_res_t = 1. / sim_h_res;
                float sim_v_res_t = 1. / sim_v_res;
                tmp.layer_count =
                    (tmp.vertical_angle - input.min_Z2) * sim_v_res_t;
                tmp.pc_idx = tmp.horizon_angle * sim_h_res_t;
                fb_list_.push_back(tmp);
            }
        } while (0);
        return ret;
    }
    void doCopyProcess(const Lidar *lidar,
                       sim_data_flow::WBPointCloud &point_cloud) {
        auto layers = lidar->getNumberOfLayers();
        auto number = lidar->getNumberOfPoints();
        auto npl = number / layers;
        point_cloud.clear_point_cloud();
        for (const auto &unit : fb_list_) {
            if (unit.layer_count < 0 || unit.pc_idx < 0)
                continue;
            if (unit.layer_count >= layers || unit.pc_idx >= npl)
                continue;
            auto pts = lidar->getLayerPointCloud(unit.layer_count);
            double x = pts[unit.pc_idx].x;
            double y = pts[unit.pc_idx].y;
            double z = pts[unit.pc_idx].z;
            if (std::abs(x) != INFINITY && std::abs(y) != INFINITY &&
                std::abs(z) != INFINITY) {
                sim_data_flow::LidarPoint *point =
                    point_cloud.add_point_cloud();
                point->set_x(x);
                point->set_y(y);
                point->set_z(z);
                point->set_time(pts[unit.pc_idx].time);
                point->set_layer_id(pts[unit.pc_idx].layer_id);
            }
        }
        point_cloud.set_size_of_each_layer(-1);
        point_cloud.set_size_of_layer(-1);
        point_cloud.set_size_of_point_cloud(point_cloud.point_cloud().size());
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
};
}  // namespace VNSim

#endif