#ifndef __FB_LOADER__H__
#define __FB_LOADER__H__

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
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

class FB_Loader {
   private:
    std::vector<struct FBContain> fb_list_;

   public:
    FB_Loader() {}
    ~FB_Loader() {}
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
    const std::vector<struct FBContain> *getInfo() { return &fb_list_; }

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