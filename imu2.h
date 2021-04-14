#pragma once
#include "ThisThread.h"
#include "broadcastqueue.h"
#include "math_types.h"
extern "C" {
#include "sh2.h"
}
#include <array>
#include <cstdio>
#include <mbed.h>
#include <mstd_memory>

inline void sh2_check(char const *file, int line, int result) {
  if (result != 0) {
    debug("sh2 call failed at %s, %d: %d", file, line, result);
  }
}

#define SH2_CHECK(value) sh2_check(__FILE__, __LINE__, value)

mstd::unique_ptr<sh2_Hal_t> make_sh2_hal(I2C *i2c);

struct IMUFrame {
  Quaternion angularOrientation;
  Vec3 linearAcceleration;
};

bool open_imu(I2C *i2c);

class IMUManager {
  Thread thread;
  BroadcastQueue<IMUFrame> broadcastqueue;
  std::unique_ptr<sh2_Hal_t> hal;

public:
  static IMUManager *get() {
    static IMUManager instance = {};
    return &instance;
  };

  bool is_running();
  void set_interface(I2C *i2c);

  bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us) {
    sh2_SensorConfig_t config = {};

    config.reportInterval_us = interval_us;
    SH2_CHECK(sh2_setSensorConfig(sensorId, &config));
    return true;
  };

  bool use_i2c(I2C *i2c);
  bool set_i2c(I2C *i2c);
  size_t add_listener(const Event<void(IMUFrame)> &ev) {
    return broadcastqueue.subscribe(ev);
  }
  void remove_listener(size_t ix) { return broadcastqueue.unsubscribe(ix); };

protected:
  static void sensorcallback(void *cookie, sh2_SensorEvent_t *pEvent) { // todo
    auto this_ = (IMUManager *)cookie;
    debug("sensor callback");
  }
  static void eventcallback(void *cookie, sh2_AsyncEvent_t *pEvent0) { /*todo*/
    auto this_ = (IMUManager *)cookie;
    debug("event callback");
  }

  IMUManager();
};
