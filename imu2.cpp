#include "imu2.h"
extern "C" {
#include "sh2.h"
#include "sh2_err.h"
}
#include <array>
#include <memory>

class SH2Hal;

void sh2_check(char const *file, int line, int result) {
  if (result != 0) {

    debug("\nsh2 call failed at %s:%d  %d\n", file, line, result);
  }
}

mstd::unique_ptr<sh2_Hal_t>(HAL) = nullptr;

bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us);

void sh2_imu_callback(void *cookie, sh2_SensorEvent_t *pEvent);

void sh2_service_task() {
  while (true) {
    sh2_service();
    ThisThread::sleep_for(10ms);
  }
};

bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us) {
  sh2_SensorConfig_t config = {};

  config.reportInterval_us = interval_us;
  SH2_CHECK(sh2_setSensorConfig(sensorId, &config));
  return true;
};

IMUManager::IMUManager() = default;
void IMUManager::set_interface(I2C *i2c) {
  if (hal) {
    thread.terminate();
    sh2_close();
  }
  hal = make_sh2_hal(i2c);
  SH2_CHECK(sh2_open(hal.get(), &IMUManager::eventcallback, (void *)this));
  SH2_CHECK(sh2_setSensorCallback(&IMUManager::sensorcallback, (void *)this));
  thread.start([this]() { sh2_service_task(); });
  ThisThread::sleep_for(500ms);
  sh2_ProductIds_t p = {0};
  // todo: Figure out why these next 3 lines are failing with -2 (SH2_ERR_BAD_PARAM)
  SH2_CHECK(sh2_getProdIds(&p));
  enableReport(SH2_LINEAR_ACCELERATION, 500000);
  enableReport(SH2_GYROSCOPE_CALIBRATED, 500000);
};
extern "C" {
void IMUManager::sensorcallback(void *cookie,
                                sh2_SensorEvent_t *pEvent) { // todo
  auto this_ = (IMUManager *)cookie;
  debug("sensor callback: %d\n", pEvent->reportId);
}
void IMUManager::eventcallback(void *cookie,
                               sh2_AsyncEvent_t *pEvent) { /*todo*/
  auto this_ = (IMUManager *)cookie;
  const char *msg;
  switch (pEvent->eventId) {
  case SH2_RESET: {
    msg = "RESET_EVENT";
    break;
  }
  case SH2_SHTP_EVENT: {
    const auto &ev = pEvent->shtpEvent;
    msg = "SH2_SHTP_EVENT";
    break;
  }
  case SH2_GET_FEATURE_RESP: {
    const auto &ev = pEvent->sh2SensorConfigResp;
    msg = "FEATURE_RESP";
    break;
  }
  default:
    msg = "UNKNOWN";
    break;
  }
  //   debug("event callback %d: %s\n", pEvent->eventId, msg);
}
}

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