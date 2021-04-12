#pragma once
#include "broadcastqueue.h"
#include "math_types.h"
#include <array>
#include <mbed.h>
#include <mstd_memory>

namespace bno08x {
static std::array<char, 2> prod_id{0xF9, 0};

inline void get_dev_info(I2C *i2c) {
  i2c->stop();
  i2c->start();
  auto x = i2c->write(0x4A<<1, prod_id.data(), prod_id.size(), true);
  static std::array<char, 2> prod_id{0xF9, 0};
  static std::array<char, 100> rx_buffer;
  auto x2 = i2c->read(0x4A<<1, rx_buffer.data(), rx_buffer.size(), false);
  i2c->stop();
  printf("%d %d %s ", x, x2, rx_buffer.data());
};

// shtp protocol:
/// b0 = length lsb
/// 1: length msb
/// 2 channel
/// 3 seqnum

};

struct IMUFrame {
  Quaternion angularOrientation;
  Vec3 linearAcceleration;
};

class IMUManager {
  Thread thread;
  I2C i2c;
  BroadcastQueue<IMUFrame> broadcastqueue;

public:
  IMUManager(PinName SDA, PinName SCL)
      : i2c{SDA, SCL} {
    thread.start([this]() { thread_task(); });
  }

  size_t add_listener(const Event<void(IMUFrame)> &ev) {
    return broadcastqueue.subscribe(ev);
  }
  void remove_listener(size_t ix) { return broadcastqueue.unsubscribe(ix); };

private:
  void thread_task() {
    bno08x::get_dev_info(&i2c);
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
