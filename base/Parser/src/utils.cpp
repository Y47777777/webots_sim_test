#include <stdio.h>

#include "utils.hpp"

int16_t asint16_t(const uint8_t *buf, int len) {
  return ((int16_t)(*buf) << 8) | ((int16_t)(*(buf + 1)));
}

uint16_t asuint16_t(const uint8_t *buf, int len) {
  return ((uint16_t)(*buf) << 8) | ((uint16_t)(*(buf + 1)));
}

int32_t asint32_t(const uint8_t *buf, int len) {
  return ((int32_t)(*buf) << 24) | ((int32_t)(*(buf + 1)) << 16) |
         ((int32_t)(*(buf + 2)) << 8) | *(buf + 3);
}

uint32_t asuint32_t(const uint8_t *buf, int len) {
  return ((unsigned int) (*buf) << 24) | ((unsigned int) (*(buf + 1)) << 16) |
         ((unsigned int) (*(buf + 2)) << 8) | *(buf + 3);
}

void to2BytesFromint16_t(const int16_t *val, uint8_t *buf) {
  // buf[0] = ((*val >> 8) & 0xff);
  // buf[1] = ((*val >> 0) & 0xff);
  *buf = ((*val >> 8) & 0xff);
  *(buf + 1) = ((*val >> 0) & 0xff);
}

void to2BytesFromuint16_t(const uint16_t *val, uint8_t *buf) {
  // buf[0] = ((*val >> 8) & 0xff);
  // buf[1] = ((*val >> 0) & 0xff);
  *buf = ((*val >> 8) & 0xff);
  *(buf + 1) = ((*val >> 0) & 0xff);
}

void to4BytesFromint32_t(const int32_t *val, uint8_t *buf) {
  // buf[0] = ((*val >> 24) & 0xffff);
  // buf[1] = ((*val >> 16) & 0xffff);
  // buf[2] = ((*val >> 8) & 0xffff);
  // buf[3] = ((*val >> 0) & 0xffff);
  *buf = ((*val >> 24) & 0xffff);
  *(buf + 1) = ((*val >> 16) & 0xffff);
  *(buf + 2) = ((*val >> 8) & 0xffff);
  *(buf + 3) = ((*val >> 0) & 0xffff);
}

void to4BytesFromuint32_t(const uint32_t *val, uint8_t *buf) {
  // buf[0] = ((*val >> 24) & 0xffff);
  // buf[1] = ((*val >> 16) & 0xffff);
  // buf[2] = ((*val >> 8) & 0xffff);
  // buf[3] = ((*val >> 0) & 0xffff);
  *buf = ((*val >> 24) & 0xffff);
  *(buf + 1) = ((*val >> 16) & 0xffff);
  *(buf + 2) = ((*val >> 8) & 0xffff);
  *(buf + 3) = ((*val >> 0) & 0xffff);
}

uint16_t CalCRC(const uint8_t *buffer, int offset, uint64_t length,
                uint16_t wCrc, uint16_t wPolynom) {
  uint16_t i, j;
  for (i = 0; i < length; i++) {
    wCrc ^= buffer[offset + i];
    for (j = 0; j < 8; j++) {
      if ((wCrc & 0x0001) == 0x0001) {
        wCrc = (uint16_t)((wCrc >> 1) ^ wPolynom);
      } else {
        wCrc = (uint16_t)(wCrc >> 1);
      }
    }
  }
  return wCrc;
}