#include "imc099.h"
#include <array>
#include <mbed.h>

namespace iMotion {

struct ByteStreamLE {
  byte *iter;
  ByteStreamLE(unsigned char *iter)
      : iter(iter){};

  uint8_t pop_u8() {
    uint8_t result = *iter;
    ++iter;
    return result;
  }

  uint16_t pop_u16() {
    uint16_t result = *iter;
    ++iter;
    result |= (*iter) << 8;
    ++iter;
    return result;
  }

  void push_u8(uint8_t x) {
    *iter = x;
    ++iter;
  }

  void push_u16(uint16_t x) {
    *iter = (uint8_t)x;
    ++iter;
    *iter = (uint8_t)(x >> 8);
    ++iter;
  }
};

auto checksum(Span<byte> rawdata) -> uint16_t {
  uint16_t result = 0;
  for (auto i = 0; i < 3; i++) {
    result -= static_cast<uint16_t>(rawdata[i * 2] | rawdata[i * 2 + 1]);
  }
  return result;
};

DataFrame::DataFrame(NodeAddress node_address, Command command,
                     uint16_t dataword0, uint16_t dataword1) {
  this->node_address = node_address;
  this->command = command;
  this->dataword0 = dataword0;
  this->dataword1 = dataword1;
  this->m_checksum = expected_checksum();
}

/// create from binary data
DataFrame::DataFrame(Span<byte> sp) {
  ByteStreamLE stream(std::begin(sp));
  node_address = NodeAddress(stream.pop_u8());
  command = Command(stream.pop_u8());
  dataword0 = stream.pop_u16();
  dataword1 = stream.pop_u16();
  m_checksum = stream.pop_u16();
}

auto DataFrame::make_register_read(AnyRegister any_register) -> DataFrame {
  uint16_t dataword0 =
      uint16_t(any_register.app_id) | uint16_t(any_register.register_id << 8);
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, Command::REGISTER_READ,
          dataword0, 0x0000};
};

auto DataFrame::make_register_write(AnyRegister any_register, uint16_t value)
    -> DataFrame {
  uint16_t dataword0 =
      uint16_t(any_register.app_id) | uint16_t(any_register.register_id << 8);
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, Command::REGISTER_WRITE,
          dataword0, value};
};

auto DataFrame::make_change_control_input_mode(ControlInputMode new_mode)
    -> DataFrame {
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, Command::SELECT_INPUT_MODE,
          0x0000, uint16_t(new_mode)};
};

auto DataFrame::make_motor_control(uint16_t target_speed) -> DataFrame {
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, Command::SET_TARGET_SPEED, 0x00,
          target_speed};
};

auto DataFrame::make_clear_fault(uint16_t target_speed) -> DataFrame {
  return {NodeAddress::ALL_NODES_WITH_RESPONSE, Command::SET_TARGET_SPEED, 0x00,
          target_speed};
};

auto DataFrame::as_bytes() -> std::array<byte, BYTES_PER_MESSAGE> {
  std::array<byte, BYTES_PER_MESSAGE> result{};
  ByteStreamLE stream(std::begin(result));
  stream.push_u8((uint8_t)node_address);
  stream.push_u8((uint8_t)command);
  stream.push_u16(dataword0);
  stream.push_u16(dataword1);
  stream.push_u16(m_checksum);
  return result;
}

auto DataFrame::expected_checksum() const -> uint16_t {
  uint16_t result = 0;
  result -= (uint16_t(node_address) | uint16_t(command) << 8);
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
    broadcastqueue.broadcast(df);
  }
}

IMC099::IMC099(PinName tx_pin, PinName rx_pin)
    : serial{tx_pin, rx_pin} {
  serial.set_baud(DEFAULT_BAUD);
  serial.set_blocking(true);
  //   std::array<uint8_t, 2> buf = {0x00, 0x6C};
  //   serial.write(buf.data(), buf.size());
  volatile bool sr = serial.readable();
  volatile bool sw = serial.writable();
  read_thread.start(callback(this, &IMC099::read_thread_fn));
  write_thread.start(callback(this, &IMC099::write_thread_fn));
}

auto IMC099::add_listener(const Event<void(DataFrame)> &response_event)
    -> size_t {
  return broadcastqueue.subscribe(response_event);
}

void IMC099::remove_listener(size_t index) {
  return broadcastqueue.unsubscribe(index);
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
