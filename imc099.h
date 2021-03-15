#pragma once

#include <array>
#include <cstdint>
#include <mbed.h>

namespace iMotion {

using byte = unsigned char;

static const auto BYTES_PER_MESSAGE = 8;

enum class NodeAddress {
  ALL_NODES_NO_RESPONSE = 0x00,
  ALL_NODES_WITH_RESPONSE = 0xFF,
};

enum class AppId {
  SYSTEM_CONTROL = 0,
  MOTOR_CONTROL = 1,
  PFC = 3,
  SCRIPT = 4,
};

constexpr auto reg_sc(uint8_t index) -> uint16_t {
  assert(index < 255);
  return (uint16_t(AppId::SYSTEM_CONTROL) | (index << 8));
};

constexpr auto reg_mc(uint8_t index) -> uint16_t {
  return (uint16_t(AppId::MOTOR_CONTROL) | (index << 8));
};

enum class SystemControlRegister : uint8_t {
  PAR_PAGE_CONF = 0,
  INTERFACE_CONF0 = 2,
  INTERFACE_CONF1 = 3,
  SYS_TASK_TIME = 62,
  CPU_LOAD = 80,
  CPU_LOAD_PEAK = 84,
  FEATURE_ID_SELECT_H = 61,
  GK_CONF = 22,
  SW_VERSION = 82,
  INTERNAL_TEMP = 81,
  SYS_TASK_CONFIG = 63,
};

enum class MotorControlRegister : uint8_t {
  HW_CONFIG = 1,
  SYS_CONFIG = 2,
  ANGLE_SELECT = 3,
  CTRL_MODE_SELECT = 4,
  PWM_FREQ = 5,
  PWM_DEADTIME_R = 6,
  PWM_DEADTIME_F = 7,
  SH_DELAY = 8,
  T_MIN_PHASE_SHIFT = 9,
  T_CNT_MIN = 10,
  PWM_GUARD_BAND = 11,
  FAULT_ENABLE = 12,
  VDC_OV_LEVEL = 13,
  VDC_UV_LEVEL = 14,
  CRITICAL_OV_LEVEL = 15,
  ROTOR_LOCK_TIME = 16,
  FLUX_FAULT_TIME = 18,
  GATEKILL_FILTER_TIME = 19,
  COMP_REF = 20,
  BTS_CHARGE_TIME = 21,
  T_CATCH_SPIN = 22,
  DIRECT_START_THR = 23,
  PARK_TIME = 24,
  PARK_ANGLE = 25,
  OPENLOOP_RAMP = 26,
  IS_PULSES = 27,
  IS_DUTY = 28,
  IS_IQ_INIT = 29,
  KP_SREG = 30,
  KX_SREG = 31,
  MOTOR_LIM = 32,
  REGEN_LIM = 33,
  REGEN_SPD_THR = 34,
  LOW_SPEED_LIM = 35,
  LOW_SPEED_GAIN = 36,
  SPD_RAMP_RATE = 37,
  MIN_SPD = 38,
  RS = 39,
  L0 = 40,
  L_SLNCY = 41,
  VOLT_SCL = 42,
  PLL_KP = 43,
  PLL_KI = 44,
  PLL_FREQ_LIM = 45,
  ANG_MTPA = 46,
  FLX_TAU = 47,
  ATAN_TAU = 48,
  SPEED_SCALE_PSC = 49,
  SPEED_SCALE = 50,
  SPEED_SCALE_RCP = 51,
  SPD_FILT_BW = 52,
  PG_DELTA_ANGLE = 53,
  IFBK_SCL = 54,
  KP_IREG = 55,
  KP_IREG_D = 56,
  KX_I_RREG = 57,
  FWK_LEVEL = 58,
  FWK_KX = 59,
  FWK_CUR_RATIO = 60,
  VDQ_LIM = 61,
  ANG_DEL = 62,
  ANG_LIM = 63,
  ADQ_FILT_BW = 64,
  PWM2_PH_THR = 65,
  T_DERATING = 66,
  T_SHUTDOWN = 67,
  CMD_STOP = 68,
  CMD_START = 69,
  CMD_GAIN = 70,
  APP_CONFIG = 71,
  NODE_ADDRESS = 72,
  PRIMARY_CONTROL_LOOP = 73,
  PHASE_LOSS_LEVEL = 74,
  TRQ_COMP_GAIN = 75,
  TRQ_COMP_ANG_OFST = 76,
  TRQ_COMP_LIM = 77,
  TRQ_COMP_ON_SPEED = 78,
  TRQ_COMP_OFF_SPEED = 79,
  POLE_PAIR = 80,
  FAULT_RETRY_PERIOD = 81,
  HALL_ANGLE_OFFSET = 85,
  HALL2_FLUX_THR = 86,
  FLUX2_HALL_THR = 87,
  HALL_SAMPLE_FILTER = 88,
  HALL_SPD_FILT_BW = 89,
  HALL_TIMEOUT_PERIOD = 94,
  KP_HALL_PLL = 100,
  COMMAND = 120,
  TARGET_SPEED = 121,
  IU = 122,
  IV = 123,
  IW = 124,
  MOTOR_SPEED = 125,
  I_ALPHA = 126,
  I_BETA = 127,
  ID_REF_EXT = 128,
  IQ_REF_EXT = 129,
  VD_EXT = 130,
  VQ_EXT = 131,
  SW_FAULTS = 132,
  SEQUENCER_STATE = 133,
  FAULT_CLEAR = 134,
  FAULT_FLAGS = 135,
  VDC_RAW = 136,
  VDC_FILT = 137,
  FLUX_ANGLE = 138,
  FLX_M = 139,
  ABS_MOTOR_SPEED = 140,
  ID_FILT = 141,
  IQ_FILT = 142,
  ID_FWK = 143,
  VTH = 144,
  FLUX_ALPHA = 145,
  FLUX_BETA = 146,
  FLX_Q = 147,
  TRQ_REF = 148,
  ID = 149,
  IQ = 150,
  V_ALPHA = 151,
  V_BETA = 152,
  SPEED_ERROR = 153,
  MOTOR_CURRENT = 154,
  OPEN_LOOP_ANGLE = 155,
  VD = 156,
  VQ = 157,
  MOTOR_VOLTAGE = 158,
  TRQ_REF_EXT = 159,
  SPD_REF = 162,
  CONTROL_FREQ = 164,
  CONTROL_DUTY = 165,
  HALL_ANGLE = 167,
  HALL_MOTOR_SPEED = 168,
  FLUX_MOTOR_SPEED = 169,
  ROTOR_ANGLE = 170,
  MOTOR_STATUS = 171,
  POSITION_COUNTER = 175,
  POSITION_COUNTER_H = 176,
  HALL_STATUS = 181,
  HALL_FREQUENCY_OUT = 182,
  HALL_PLL_FREQUENCY_ADJUST = 183,
  HALL_ATAN_ANGLE = 184,
  HALL_U = 185,
  HALL_V = 186,
  IPEAK = 187,
  CURRENT_AMP_OFFSET0 = 188,
  CURRENT_AMP_OFFSET1 = 189,
  TRQ_REF_TOTAL = 190,
  TRQ_COMP_BASE_ANGLE = 191,
  TRQ_COMP_STATUS = 194,
};

enum class PFCControlRegister : uint8_t {
  PFC_HW_CONFIG = 1,
  PFC_SYS_CONFIG = 2,
  PFC_PWM_FREQ = 3,
  PFC_T_MIN_OFF = 4,
  PFC_DEADTIME = 5,
  PFC_SH_DELAY = 6,
  PFC_I_RECT_LIM = 7,
  PFC_I_GEN_LIM = 8,
  PFC_VDC_RAMP_RATE = 9,
  PFC_KP_VREG = 10,
  PFC_KX_VREG = 11,
  PFC_KP_IREG = 12,
  PFC_KX_IREG = 13,
  PFC_TRACKING_CYCLE = 15,
  PFC_TRACKING_GAIN = 16,
  PFC_HALF_CYCLE_MIN = 17,
  PFC_HALF_CYCLE_MAX = 18,
  PFC_VAC_ZC_THR = 19,
  PFC_VAC_OV_LEVEL = 20,
  PFC_VAC_UV_LEVEL = 21,
  PFC_VDC_OV_LEVEL = 22,
  PFC_VDC_UV_LEVEL = 23,
  PFC_AC_DC_SCALE = 24,
  PFC_L_FACTOR = 25,
  PFC_FAULT_ENABLE = 26,
  PFC_GATE_KILL_TIME = 27,
  PFC_TARGET_DC_VOLT = 28,
  PFC_SEQUENCER_STATE = 81,
  PFC_COMMAND = 82,
  PFC_FAULT_CLEAR = 85,
  PFC_SW_FAULTS = 87,
  PFC_TARGET_VOLT = 89,
  PFC_VOLTAGE_P_IOUTPUT = 90,
  PFC_VDC_RAW = 92,
  PFC_IPFC_RAW = 93,
  PFC_ABS_VAC_RAW = 94,
  PFC_VAC_RMS = 98,
  PFC_VDC_FILT = 99,
  PFC_VAC_RAW = 103,
  PFC_FAULT_FLAG = 104,
  PFC_IPFC_AVG = 105,
  PFC_IPFC_RMS = 106,
  PFC_AC_POWER = 107,
  PFC_CURRENT_P_IOUTPUT = 110,
};

enum class ScriptRegister {
  // todo
};

struct AnyRegister {
  AppId app_id;
  uint8_t register_id;
  AnyRegister(uint16_t dataword0)
      : app_id{AppId(uint8_t(dataword0))},
        register_id{uint8_t(dataword0 >> 8)} {};
  AnyRegister(SystemControlRegister r)
      : app_id(AppId::SYSTEM_CONTROL), register_id(uint8_t(r)){};
  AnyRegister(MotorControlRegister r)
      : app_id(AppId::MOTOR_CONTROL), register_id(uint8_t(r)){};
  AnyRegister(PFCControlRegister r)
      : app_id(AppId::PFC), register_id(uint8_t(r)){};
  AnyRegister(ScriptRegister r)
      : app_id(AppId::SCRIPT), register_id(uint8_t(r)){};
};

enum class UartCommand {
  IS_REPLY_FLAG = 0x80,

  READ_STATUS = 0x00,
  READ_STATUS_REPLY = IS_REPLY_FLAG | READ_STATUS,
  CLEAR_FAULT_FLAG = 0x01,
  CLEAR_FAULT_FLAG_REPLY = IS_REPLY_FLAG | CLEAR_FAULT_FLAG,
  SELECT_INPUT_MODE = 0x02,
  SELECT_INPUT_MODE_REPLY = IS_REPLY_FLAG | SELECT_INPUT_MODE,
  SET_TARGET_SPEED = 0x03,
  SET_TARGET_SPEED_REPLY = IS_REPLY_FLAG | SET_TARGET_SPEED,
  REGISTER_READ = 0x05,
  REGISTER_READ_REPLY = IS_REPLY_FLAG | REGISTER_READ,
  REGISTER_WRITE = 0x06,
  REGISTER_WRITE_REPLY = IS_REPLY_FLAG | REGISTER_WRITE,
  LOAD_SAVE_PARAM_SET = 0x20,
  LOAD_SAVE_PARAM_SET_REPLY = IS_REPLY_FLAG | LOAD_SAVE_PARAM_SET,
};

enum class StatusCode {
  FAULT_FLAGS = 0x0000,
  MOTOR_SPEED = 0x0001,
  MOTOR_STATE = 0x0002,
  NODE_ID = 0x0003
};

enum class ControlInputMode {
  UART = 0,
  ANALOG = 1,
  FREQ = 2,
  DUTY = 3,
};

struct DataFrame {
  byte node_address{};
  byte command{};
  uint16_t dataword0{};
  uint16_t dataword1{};
  uint16_t m_checksum;

  DataFrame(NodeAddress node_address, UartCommand command, uint16_t dataword0,
            uint16_t dataword1);

  /// create from binary data
  DataFrame(Span<byte> sp);

  // factory methods:
  static auto make_register_read(AnyRegister any_register) -> DataFrame;

  static auto make_register_write(AnyRegister any_register, uint16_t value)
      -> DataFrame;

  static auto make_change_control_input_mode(ControlInputMode new_mode)
      -> DataFrame;

  static auto make_motor_control(uint16_t target_speed) -> DataFrame;

  static auto make_clear_fault(uint16_t target_speed) -> DataFrame;

  auto as_bytes() -> std::array<byte, BYTES_PER_MESSAGE>;
  auto expected_checksum() const -> uint16_t;
};

class IMC099 {
  static const auto DEFAULT_BAUD = 1000000;
  static const auto MAX_PENDING = 4;
  static const auto MAX_LISTENERS = 8;

  BufferedSerial serial;

  std::array<mstd::unique_ptr<Event<void(DataFrame)>>, MAX_LISTENERS> listeners;
  Mail<DataFrame, MAX_PENDING> frames_to_write;
  Thread write_thread{};
  Thread read_thread{};

  /// throw away the rest of the input buffer
  void flush_read();

  /// thread responsible for writing to the device
  void write_thread_fn();

  /// thread responsible for reading from the device
  void read_thread_fn();

public:
  IMC099(PinName tx_pin, PinName rx_pin);

  /// Submit an event to be triggered for every received message
  auto add_listener(Event<void(DataFrame)> &response_event) -> size_t;

  /// Remove an event so that it is no longer triggered on new messages
  void remove_listener(size_t index);

  /// write the dataframe to the device
  bool send(DataFrame request);
};
}
