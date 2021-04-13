#include "imu2.h"
#include "sh2.h"
#include "sh2_err.h"
#include <array>
#include <memory>

int sh2_i2c_open(sh2_Hal_t *self);
void sh2_i2c_close(sh2_Hal_t *self);
int sh2_i2c_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                 uint32_t *t_us);
int sh2_i2c_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
uint32_t sh2_i2c_getTimeUs(sh2_Hal_t *self) { return us_ticker_read(); };

struct Sh2ImplIMUI2C : public sh2_Hal_t {
public:
  I2C *i2c;
  explicit Sh2ImplIMUI2C(I2C *i2c_)
      : sh2_Hal_t() {
    this->open = sh2_i2c_open;
    this->close = sh2_i2c_close;
    this->read = sh2_i2c_read;
    this->write = sh2_i2c_write;
    this->getTimeUs = sh2_i2c_getTimeUs;
    this->i2c = i2c_;
  }
};

mstd::unique_ptr<sh2_Hal_t>(HAL) = nullptr;

bool open_imu(unique_ptr<sh2_Hal_t> hal) {
  static unique_ptr<sh2_Hal_t> s_hal;
  if (s_hal) {
    sh2_i2c_close(s_hal.get());
  }

  s_hal = mstd::move(hal);
  auto status = sh2_i2c_open(s_hal.get());
  if (status != SH2_OK) {
    debug("sh2_i2c_open failed with status %d", status);
    return false;
  }
  return true;
};

bool open_imu(I2C *i2c) { return open_imu(mstd::make_unique<sh2_Hal_t>(i2c)); };

bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us);

void sh2_imu_callback(void *cookie, sh2_SensorEvent_t *pEvent);

mstd::unique_ptr<sh2_Hal_t> make_sh2_hal_i2c(I2C *i2c) {
  auto hal = mstd::make_unique<Sh2ImplIMUI2C>(i2c);
  hal->open = sh2_i2c_open;
  hal->close = sh2_i2c_close;
  hal->read = sh2_i2c_read;
  hal->write = sh2_i2c_write;
  hal->getTimeUs = sh2_i2c_getTimeUs;
  hal->i2c = i2c;
  return hal;
};

void sh2_service_task() {
  while (true) {
    sh2_service();
    ThisThread::sleep_for(10ms);
  }
};

// typedef void (sh2_SensorCallback_t)(void * cookie, sh2_SensorEvent_t
// *pEvent);

SH2Imu::SH2Imu(I2C *i2c)
    : hal(nullptr) {
  hal = make_sh2_hal_i2c(i2c);
  if (sh2_open(this->hal.get(), nullptr, nullptr) != SH2_OK) {
    debug("failed to open SH2");
    return;
  }

  sh2_ProductIds_t p;
  auto status = sh2_getProdIds(&p);
  if (status != 0) {
    debug("failed to communicate with SH2: %d", status);
  } else {
    debug("opened SH2 successfully");
  }
  sh2_setSensorCallback(&sh2_imu_callback, this);
  enableReport(SH2_LINEAR_ACCELERATION, 500000);
  enableReport(SH2_GYROSCOPE_CALIBRATED, 500000);

  t.start([this]() { sh2_service_task(); });
};

bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us) {
  sh2_SensorConfig_t config = {};

  config.reportInterval_us = interval_us;
  auto status = sh2_setSensorConfig(sensorId, &config);
  return status == SH2_OK;
};

void sh2_imu_callback(void *cookie, sh2_SensorEvent_t *pEvent) {
  auto this_ = (SH2Imu *)cookie;
  debug("Sensor callback received");
}

const uint8_t ADDR = 0x4A << 1;
int sh2_i2c_open(sh2_Hal_t *self) {
  auto &mb = *((Sh2ImplIMUI2C *)self);
  // Serial.println("I2C HAL open");
  mb.i2c->stop();
  // send a software reset

  const std::array<char, 5> softreset_pkt{5, 0, 1, 0, 1};
  auto did_fail =
      mb.i2c->write(ADDR, softreset_pkt.data(), softreset_pkt.size(), false);
  if (did_fail != 0) {
    return -1;
  }
  return 0;
};

void sh2_i2c_close(sh2_Hal_t *self){};

static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
  auto &mb = (static_cast<Sh2ImplIMUI2C &>(*self));
  std::array<char, 2> header;
  auto rc = mb.i2c->read(ADDR, header.data(), header.size(), true);
  if (rc != SH2_OK) {
    mb.i2c->stop();
    return 0;
  }
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  if (packet_size == 65535) {
    mb.i2c->stop();
    return 0;
  }
  packet_size &= ~0x8000;
  if (packet_size > len) {
    mb.i2c->stop();
    return 0;
  }
  auto rc2 = mb.i2c->read(ADDR, (char *)pBuffer, header.size(), true);
  if (rc2 != SH2_OK) {
    mb.i2c->stop();
    return 0;
  }
  *t_us = us_ticker_read();
  return packet_size;
};

int sh2_i2c_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  auto &mb = (static_cast<Sh2ImplIMUI2C &>(*self));
  int ilen = int(len);
  auto rc = mb.i2c->write(ADDR, (const char *)pBuffer, ilen);
  if (rc != SH2_OK) {
    return 0;
  }
  return ilen;
}
