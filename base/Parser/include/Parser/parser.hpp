#ifndef _PARSER_HPP_
#define _PARSER_HPP_
#include <stdio.h>

#include <vector>

struct Package {
  uint8_t *buf;
  int len;
};

struct UpdateValue {
  const void *val;
  int len;  // size of the bytes
  // 1. subId are used for WheelCoder, 0 means left, 1 means right
  // 2. if no more than 1 param, give anything for subId
  int subId;
};

class InputDecoder {
 public:
  InputDecoder();
  ~InputDecoder();

  /// @brief load Actuators.config
  /// @param path
  /// @return error code
  /// @note 1. 0 means successful
  ///       2. -1 means not config not loaded
  // int load(const char* path); // TODO: loadConfig
  int loadConfig(const char *path);

  // /// @brief get node handle by node keyword
  // /// @param key node keyword
  // /// @return node handle
  // int getHandle(const char* key);

  /// @brief parse frame
  /// @param package frame info
  /// @return error code
  /// @note 1. 0 means successful
  ///       2. -1 means not config not loaded
  int decodePackage(const struct Package *package);

  // /// @brief get origin coder or value by  // TODO: combine getHandle and
  // getValue, solveValue
  // /// @param handle node handle
  // /// @param value output value
  // /// @return
  // /// @note 1. 0 means successful
  // ///       2. -1 means not config not loaded
  // ///       3. You need to give proper size value, FX: int32_t or int16_t
  // int getValue(int handle, void* value);

  // /// @brief transform value for each node based on Actuators.config
  // /// @return error code/*  */
  // /// @note 1. 0 means successful
  // ///       2. -1 means not config not loaded
  // // TODO: template ???
  // int solveValue(int handle, float input, double* output); // TODO: int
  // getValue(const char* key, void* o_value, double* output); // 0 for not use
  int getValue(const char *key, void *direct_output_value,
               double *calculated_output);
};

class OutputEncoder {
 public:
  OutputEncoder();
  ~OutputEncoder();

  /// @brief load Sencers.config
  /// @param path
  /// @return
  /// @note 1. 0 means successful
  ///       2. -1 means not config not loaded
  int loadConfig(const char *path);

  // /// @brief get node handle by node keyword
  // /// @param key node keyword
  // /// @return node handle
  // int getHandle(const char* key);

  // /// @brief update transformed or bytes value to each node
  // /// @param handle node handle
  // /// @param val_list input value list for each node
  // /// @return error code
  // /// @note 1. 0 means successful
  // ///       2. -1 means not config not loaded
  // int updateValue(int handle, std::vector<struct UpdateValue*>& val_list); //
  // int updateValue(const char* key, const std::vector<double>& input,
  // std::vector<struct UpdateValue*>& param_list)

  // /// @brief transform value for each node based on Sensors.config
  // /// @param handle node handle
  // /// @param input origin input
  // /// @param output tranformed output
  // /// @return error code
  // /// @note 1. 0 means successful
  // ///       2. -1 means not config not loaded
  // int solveValue(int handle, const std::vector<double>& input,
  // std::vector<double>& output);
  int updateValue(const char *key, const std::vector<double> &calculated_input,
                  const std::vector<struct UpdateValue *> &direct_input);

  /// @brief get n bytes frame
  /// @return frame info
  const struct Package *encodePackage();
};

#endif