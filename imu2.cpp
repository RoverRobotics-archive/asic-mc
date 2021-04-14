#include "imu2.h"
extern "C" {
#include "sh2.h"
#include "sh2_err.h"
}
#include <array>
#include <memory>

class SH2Hal;

mstd::unique_ptr<sh2_Hal_t>(HAL) = nullptr;

bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us);

void sh2_imu_callback(void *cookie, sh2_SensorEvent_t *pEvent);

void sh2_service_task() {
  while (true) {
    sh2_service();
    ThisThread::sleep_for(10ms);
  }
};
IMUManager ::IMUManager() {
  thread.start([this]() { sh2_service_task(); });
}

void IMUManager::set_interface(I2C *i2c) {
  if (hal) {
    sh2_close();
  }
  hal = make_sh2_hal(i2c);
  SH2_CHECK(sh2_open(hal.get(), &IMUManager::eventcallback, (void *)this));
  SH2_CHECK(sh2_setSensorCallback(&IMUManager::sensorcallback, (void *)this));

  sh2_ProductIds_t p;
  SH2_CHECK(sh2_getProdIds(&p));
  enableReport(SH2_LINEAR_ACCELERATION, 500000);
  enableReport(SH2_GYROSCOPE_CALIBRATED, 500000);
};

void thread_task() {
  while (true) {
    sh2_service();
    ThisThread::sleep_for(10ms);
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