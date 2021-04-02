#pragma once
#include "BNO055.h"
#include "broadcastqueue.h"
#include "math_types.h"
#include <array>
#include <mbed.h>
#include <mstd_memory>

struct IMUFrame {
  Quaternion angularOrientation;
  Vec3 linearAcceleration;
};

const size_t NUM_IMU_LISTENERS = 10;
class IMUManager {
  Thread thread;
  BNO055 dev;
  BroadcastQueue<IMUFrame> broadcastqueue;

public:
  IMUManager(PinName SDA, PinName SCL)
      : dev{SDA, SCL} {
    thread.start([this]() { thread_task(); });
  }

  size_t add_listener(const Event<void(IMUFrame)> &ev) {
    return broadcastqueue.subscribe(ev);
  }
  void remove_listener(size_t ix) { return broadcastqueue.unsubscribe(ix); };

private:
  void thread_task() {
    dev.reset();
    while (!dev.check()) {
      ThisThread::sleep_for(1ms);
    }

    // can get:
    // absolute orientation
    // angular velocity
    // acceleration vector
    // gravity vector
    dev.setmode(OPERATION_MODE_NDOF);
    while (true) {
      // dev.get_accel();
      // dev.get_angles();
      // dev.get_calib();
      // dev.get_grv();
      // dev.get_gyro();
      dev.get_lia();
      // dev.get_mag();
      dev.get_quat();
      // dev.get_temp();
      IMUFrame fr;
      fr.angularOrientation = {dev.quat.w,
                               {dev.quat.x, dev.quat.y, dev.quat.z}};
      fr.linearAcceleration = {dev.lia.x, dev.lia.y, dev.lia.z};
      broadcastqueue.broadcast(fr);
    }
  }
};
