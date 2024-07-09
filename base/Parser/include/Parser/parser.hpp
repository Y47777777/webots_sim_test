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
    // 2. if no more than 1 params, give anything for subId
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

    /// @brief parse frame
    /// @param package frame info
    /// @return error code
    /// @note 1. 0 means successful
    ///       2. -1 means not config not loaded
    int decodePackage(const struct Package *package);

    /// @brief transform value for each node based on Actuators.config
    /// @return error code/*  */
    /// @note 1. 0 means successful
    ///       2. -1 means not config not loaded
    int getValue(const char *key, double *output, const char *func_key = "");

    /// @brief get value from origin bytes (uint16_t , int16_t, uint32_t,
    /// int32_t)
    /// @param key key name
    /// @param output output value
    /// @param len bytes size of the valuable
    /// @return
    int getValue2(const char *key, void *output, int size,
                  const char *func_key = "");

    /// @brief get value from switch bits value
    /// @param key
    /// @param bits
    /// @param output
    /// @return
    int getSwitchValue(const char *key, int bits, bool *output,
                       const char *func_key = "");
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

    /// @brief transform value for each node based on Sensors.config
    /// @param handle node handle
    /// @param input origin input
    /// @param output tranformed output
    /// @return error code
    /// @note 1. 0 means successful
    ///       2. -1 means not config not loaded
    int updateValue(const char *key, int len, const char *func_key = "", ...);

    /// @brief update bytes value to each node
    /// @param key
    /// @param size
    /// @param input, uint16_t, int16_t, int32_t, uint32_t value
    /// @return
    int updateValue2(const char *key, const void *input, int size,
                     const char *func_key = "");

    /// @brief update bits value
    /// @param key key name
    /// @param values bits position
    /// @return error code
    /// @note: 1. 0 means successful
    ///        2. -1 means invalid key
    ///        3. -2 means invalid bits position
    int updateSwitchValue(const char *key, int bits, bool value,
                          const char *func_key = "");

    /// @brief get n bytes frame
    /// @return frame info
    const struct Package *encodePackage();
};

#endif