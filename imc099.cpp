#include "imc099.h"
#include <mbed.h>
#include <common_functions.h>
#include <mstd_memory>
#include <array>

namespace iMotion {

auto checksum(Span<byte> rawdata) -> uint16_t {
  uint16_t result = 0;
  for (auto i = 0; i < 3; i++) {
    result -= static_cast<uint16_t>(rawdata[i * 2] | rawdata[i * 2 + 1]);
  }
  return result;
};

DataFrame::DataFrame(NodeAddress node_address, UartCommand command,
                     uint16_t dataword0, uint16_t dataword1)
     {
this->node_address = (uint8_t)node_address;
this->command = (uint8_t)command;
this->dataword0=dataword0;
this->dataword1=dataword1;
         this->m_checksum = expected_checksum();
     }

/// create from binary data
DataFrame::DataFrame(Span<byte> sp) {
  auto it = std::begin(sp);
  node_address = *it;
  it++;
  command = *it;
  it++;
  dataword0 = common_read_16_bit_inverse(it);
  it += 2;
  dataword1 = common_read_16_bit_inverse(it);
  it += 2;
  m_checksum = common_read_16_bit_inverse(it);
  it += 2;
}

auto DataFrame::make_register_read(AnyRegister any_register) -> DataFrame {
  uint16_t dataword0 =
      uint16_t(any_register.app_id) | uint16_t(any_register.register_id << 8);
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, UartCommand::REGISTER_READ,
          dataword0, 0x0000};
};

auto DataFrame::make_register_write(AnyRegister any_register, uint16_t value)
    -> DataFrame {
  uint16_t dataword0 =
      uint16_t(any_register.app_id) | uint16_t(any_register.register_id << 8);
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, UartCommand::REGISTER_WRITE,
          dataword0, value};
};

auto DataFrame::make_change_control_input_mode(ControlInputMode new_mode)
    -> DataFrame {
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, UartCommand::SELECT_INPUT_MODE,
          0x0000, uint16_t(new_mode)};
};

auto DataFrame::make_motor_control(uint16_t target_speed) -> DataFrame {
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, UartCommand::SET_TARGET_SPEED,
          0x00, target_speed};
};

auto DataFrame::make_clear_fault(uint16_t target_speed) -> DataFrame {
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, UartCommand::SET_TARGET_SPEED,
          0x00, target_speed};
};

auto DataFrame::as_bytes() -> std::array<byte, BYTES_PER_MESSAGE> {
  std::array<byte, BYTES_PER_MESSAGE> result{};
  auto it = std::begin(result);
  *it++ = node_address;
  *it++ = command;
  it = common_write_16_bit_inverse(dataword0, it);
  it = common_write_16_bit_inverse(dataword1, it);
  it = common_write_16_bit_inverse(m_checksum, it);
  return result;
}

auto DataFrame::expected_checksum() const -> uint16_t {
  uint16_t result = 0;
  result -= (node_address | command << 8);
  result -= dataword0;
  result -= dataword1;
  return result;
};

void IMC099::flush_read() {
  char ign = 0;
  while (serial.readable()) {
    serial.read(&ign, 1);
  }
}

void IMC099::write_thread_fn() {
  while (true) {
    DataFrame *df = frames_to_write.try_get_for(Kernel::wait_for_u32_forever);
    auto raw = df->as_bytes();
    frames_to_write.free(df);
    serial.write(raw.data(), raw.size());
  }
}

void IMC099::read_thread_fn() {
  while (true) {
    size_t bytes_read = 0;
    std::array<byte, BYTES_PER_MESSAGE> buf = {};

    while (bytes_read < BYTES_PER_MESSAGE) {
      bytes_read +=
          serial.read(&buf[bytes_read], BYTES_PER_MESSAGE - bytes_read);
    }
    DataFrame df{make_Span(buf.data(), buf.size())};

    if (df.expected_checksum() != df.expected_checksum()) {
      flush_read();
      continue;
    }

    for (auto &listener : listeners) {
      if (listener) {
        listener->post(df);
      }
    }
  }
}

IMC099::IMC099(PinName tx_pin, PinName rx_pin) : serial{tx_pin, rx_pin} {
  serial.set_baud(DEFAULT_BAUD);
  serial.set_blocking(true);
  std::array<uint8_t,2> buf = {0x00,0x6C};
  serial.write(buf.data(), buf.size());
  volatile bool sr = serial.readable();
  volatile bool sw = serial.writable();
  read_thread.start(callback(this, &IMC099::read_thread_fn));
  write_thread.start(callback(this, &IMC099::write_thread_fn));
}

auto IMC099::add_listener(Event<void(DataFrame)> &response_event) -> size_t {
  for (auto i = 0; i < MAX_LISTENERS; ++i) {
    if (!listeners[i]) {
      listeners[i] = mstd::make_unique<Event<void(DataFrame)>>(response_event);
      return i;
    }
  }
  return -1;
}

void IMC099::remove_listener(size_t index) {
  MBED_ASSERT(index < MAX_LISTENERS);
  listeners[index].reset();
}

bool IMC099::send(DataFrame request) {
  auto msg_ptr = frames_to_write.try_alloc();
  if (!msg_ptr)
    return false;
  (*msg_ptr) = request;
  frames_to_write.put(msg_ptr);
  return true;
}
};
