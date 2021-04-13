#pragma once
#include "ThisThread.h"
#include "broadcastqueue.h"
#include "math_types.h"
#include <array>
#include <assert.h>
#include <cstdio>
#include <mbed.h>
#include <mstd_memory>

class BNO08x {};

namespace bno08x {
const std::array<char, 2> prod_id{0xF9, 0};
extern std::array<char, 100> rx_buffer;

extern void on_dev_info(int);
inline void get_dev_info(I2C *i2c) {
  //   i2c->abort_transfer();
  //   ThisThread::sleep_for(5ms);
  //   bool did_fail =
  //       i2c->transfer(0x4A << 1, prod_id.data(), prod_id.size(),
  //       rx_buffer.data(),
  //                     rx_buffer.size(), on_dev_info, I2C_EVENT_ALL);
  //   MBED_ASSERT(!did_fail);
  i2c->stop();
  ThisThread::sleep_for(5ms);
  i2c->stop();
  int x = -1;
  do {
    x = i2c->write(0x4A << 1, prod_id.data(), prod_id.size(), true);

    ThisThread::sleep_for(1ms);
  } while (x == -2);
  auto x2 = i2c->read(0x4A << 1, rx_buffer.data(), rx_buffer.size(), false);

  uint16_t shtp_len = rx_buffer[0] | rx_buffer[1] << 8;
  uint8_t shtp_channel = rx_buffer[2];
  uint8_t shtp_seqnum = rx_buffer[3];
  debug(" %d %d %s \n\n\n\n", x, x2, rx_buffer.data());
};
};
// shtp protocol:
/// b0 = length lsb
/// 1: length msb
/// 2 channel
/// 3 seqnum

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
    while (true) {
      i2c.frequency(100000);
      bno08x::get_dev_info(&i2c);
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
