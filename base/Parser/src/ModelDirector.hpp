#ifndef _MODELDIRECTOR_HPP_
#define _MODELDIRECTOR_HPP_
#include <map>
#include <memory>

#include "Parser/parser.hpp"
#include "models.hpp"

class ActuatorDirector {
   private:
    int total_;
    std::vector<std::shared_ptr<class AcuratorModel>> package_set_;
    std::map<std::string, int> support_handle_;

   public:
    ActuatorDirector();
    ~ActuatorDirector();
    void create(const char *name, const char *function,
                const std::vector<struct SpecialParam> &param, int length = -1,
                bool is32bits = false);
    void decode(uint8_t *buf, int length);
    int get(const char *name, const char *func_key);
    std::shared_ptr<class AcuratorModel> get(int handle);
    // void solveValue(int handle, float input, double* output);
};

class SencerDirector {
   private:
    std::vector<std::shared_ptr<class SensorModel>> package_set_;
    std::map<std::string, int> support_handle_;
    int total_;
    int CRC_Handle_;
    struct Package package_;

   public:
    SencerDirector();
    ~SencerDirector();
    void create(const char *name, const char *function,
                const std::vector<struct SpecialParam> &param, int length = -1,
                bool is32bits = false);
    void encode(int handle, std::vector<struct UpdateValue *> &val_list);
    // void solveValue(int handle, const std::vector<double>& input,
    // std::vector<double>& output);
    int get(const char *name, const char *func_key);
    std::shared_ptr<class SensorModel> get(int handle);
    const struct Package *get();
};

#endif