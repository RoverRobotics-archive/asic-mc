#pragma once
#include "ThisThread.h"
#include "broadcastqueue.h"
#include "math_types.h"
#include <array>
#include <assert.h>
#include <cstdio>
#include <mbed.h>
#include <mstd_memory>

extern "C" {
struct sh2_Hal_s;
typedef struct sh2_Hal_s sh2_Hal_t;
};

mstd::unique_ptr<struct sh2_Hal_s> make_sh2_hal_i2c(I2C *i2c);

class SH2Imu {
  Thread t;
  mstd::unique_ptr<struct sh2_Hal_s> hal;

  explicit SH2Imu(I2C *i2c){

  };
};

struct IMUFrame {
  Quaternion angularOrientation;
  Vec3 linearAcceleration;
};

bool open_imu(I2C *i2c);

class IMUManager {
  Thread thread;
  BroadcastQueue<IMUFrame> broadcastqueue;
  SH2Imu imu;

public:
  IMUManager(I2C *i2c) {
    open_imu(i2c);
    imu = make_sh2_hal_i2c(&i2c);
    thread.start([this]() { thread_task(); });
  }
  size_t add_listener(const Event<void(IMUFrame)> &ev) {
    return broadcastqueue.subscribe(ev);
  }
  void remove_listener(size_t ix) { return broadcastqueue.unsubscribe(ix); };

private:
  void thread_task() {
    while (true) {
      //   i2c.frequency(100000);
      //   bno08x::get_dev_info(&i2c);
      ThisThread::sleep_for(1s);
    }
    // dev.reset();
    // while (!dev.check()) {
    //   ThisThread::sleep_for(1ms);
    // }

    // // can get:
    // // absolute orientation
    // // angular velocity
    // // acceleration vector
    // // gravity vector
    // dev.setmode(OPERATION_MODE_NDOF);
    // debug("IMU software v%d.%d", dev.ID.sw[1], dev.ID.sw[0]);
    // while (true) {
    //   // dev.get_accel();
    //   // dev.get_angles();
    //   // dev.get_calib();
    //   // dev.get_grv();
    //   // dev.get_gyro();
    //   dev.get_lia();
    //   // dev.get_mag();
    //   dev.get_quat();
    //   // dev.get_temp();
    //   IMUFrame fr;
    //   fr.angularOrientation = {dev.quat.w,
    //                            {dev.quat.x, dev.quat.y, dev.quat.z}};
    //   fr.linearAcceleration = {dev.lia.x, dev.lia.y, dev.lia.z};
    //   broadcastqueue.broadcast(fr);
    // }
  }
};
