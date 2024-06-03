#ifndef _UTILS_HPP_
#define _UTILS_HPP_
#include <stdint.h>

int16_t asint16_t(const uint8_t *buf, int len);

uint16_t asuint16_t(const uint8_t *buf, int len);

int32_t asint32_t(const uint8_t *buf, int len);

uint32_t asuint32_t(const uint8_t *buf, int len);

void to2BytesFromint16_t(const int16_t *val, uint8_t *buf);

void to2BytesFromuint16_t(const uint16_t *val, uint8_t *buf);

void to4BytesFromint32_t(const int32_t *val, uint8_t *buf);

void to4BytesFromuint32_t(const uint32_t *val, uint8_t *buf);

uint16_t CalCRC(const uint8_t *buffer, int offset, uint64_t length,
                uint16_t wCrc, uint16_t wPolynom);

#endif