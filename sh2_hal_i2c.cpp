#include "imu2.h"
#include <mbed.h>

extern "C" {
#include "sh2_err.h"
#include "sh2_hal.h"
}
#include <array>

/// An implementation of the SH2 HAL interface for MBed I2C
mstd::unique_ptr<sh2_Hal_t> make_sh2_hal(I2C *i2c);

struct sh2_Hal_i2c_t : public sh2_Hal_t {
  I2C *i2c;
};

const uint8_t ADDR = 0x4A << 1;
// const std::array<char, 5> softreset_pkt{5, 0, 1, 0, 1};

// todo: move this declaration elsewhere
// DigitalOut rst_imu{PA_3};

int sh2_i2c_open(sh2_Hal_t *self) {
  auto &mb = (static_cast<sh2_Hal_i2c_t &>(*self));
  //   rst_imu.write(0);
  ThisThread::sleep_for(1ms);
  //   rst_imu.write(1);

  // Serial.println("I2C HAL open");
  // send a software reset
  //   auto did_fail =
  //       mb.i2c->write(ADDR, softreset_pkt.data(), softreset_pkt.size());

  //   if (did_fail != 0) {
  //     return -1;
  //   }
  return 0;
};

void sh2_i2c_close(sh2_Hal_t *self){};

static int sh2_i2c_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                        uint32_t *t_us) {
  for (auto i = 0; i < len; ++i) {
    pBuffer[i] = 0xff;
  }
  auto &mb = (static_cast<sh2_Hal_i2c_t &>(*self));
  auto rc = mb.i2c->read(ADDR, (char *)pBuffer, (int)len);
  if (rc) {
    return 0;
  }
  *t_us = us_ticker_read();

  uint16_t packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8;
  packet_size &= ~0x8000;
  if (packet_size > len) {
    packet_size = len;
  }
  //   auto rc2 = mb.i2c->read(ADDR, (char *)pBuffer, packet_size);

  std::array<char, 400> dat;
  for (auto i = 0; i < packet_size; ++i)
    dat[i] = pBuffer[i];
  return packet_size;
};

int sh2_i2c_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  auto &mb = (static_cast<sh2_Hal_i2c_t &>(*self));
  int ilen = int(len);
  auto rc = mb.i2c->write(ADDR, (const char *)pBuffer, ilen);
  if (rc != 0) {
    debug("i2c write finished with rc %d\n", rc);
    return 0;
  }
  return ilen;
}

uint32_t sh2_i2c_getTimeUs(sh2_Hal_t *self) { return us_ticker_read(); };

mstd::unique_ptr<sh2_Hal_t> make_sh2_hal(I2C *i2c) {
  auto hal = mstd::make_unique<sh2_Hal_i2c_t>();
  hal->open = &sh2_i2c_open;
  hal->close = &sh2_i2c_close;
  hal->read = &sh2_i2c_read;
  hal->write = &sh2_i2c_write;
  hal->getTimeUs = &sh2_i2c_getTimeUs;
  hal->i2c = i2c;
  return hal;
}