#include "imu2.h"
#include "Kernel.h"
#include "sh2_hal.h"
#include "us_ticker_api.h"
#include <array>
#include <cstdio>

int sh2_open(sh2_Hal_t *self);

void sh2_close(sh2_Hal_t *self);

int sh2_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
int sh2_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

uint32_t sh2_getTimeUs(sh2_Hal_t *self) { return us_ticker_read(); };

struct sh2_imu_mbed : public sh2_Hal_s {
  sh2_imu_mbed() {
    this->open = sh2_open;
    this->close = sh2_close;
    this->read = sh2_read;
    this->write = sh2_write;
    this->getTimeUs = sh2_getTimeUs;
  }
};