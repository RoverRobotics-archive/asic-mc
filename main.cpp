#include <mbed.h>

#include "EventQueue.h"
#include "imc099.h"

void message_received_callback(iMotion::DataFrame df) {
  switch (iMotion::UartCommand(df.command)) {
  case iMotion::UartCommand::READ_STATUS: {
    auto delement = iMotion::AnyRegister(df.dataword0);
    auto value = df.dataword1;
    break;
  }
  default: {
    return;
  }
  }
}

auto main() -> int {
  iMotion::IMC099 imc099{PA_0, PA_1};
  auto registers = {
      iMotion::SystemControlRegister::CPU_LOAD,
      iMotion::SystemControlRegister::CPU_LOAD_PEAK,
      iMotion::SystemControlRegister::SW_VERSION,
  };
  EventQueue q;

  Event<void(iMotion::DataFrame)> message_received_event =
      q.event(message_received_callback);

  imc099.add_listener(message_received_event);

  for (auto reg : registers) {
    auto msg = iMotion::DataFrame::make_register_read(reg);
    imc099.send(msg);
  }
};