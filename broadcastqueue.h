#pragma once
#include <array>
#include <mbed.h>

const size_t MAX_SUBSCRIBERS = 8;
template <typename T> class BroadcastQueue {

public:
  using EventT = Event<void(T)>;

private:
  std::array<mstd::unique_ptr<EventT>, MAX_SUBSCRIBERS> subscribers;

public:
  size_t subscribe(const EventT &subscriber) {
    for (size_t i = 0; i < subscribers.size(); ++i) {
      if (!subscribers[i]) {
        subscribers[i] = mstd::make_unique<EventT>(subscriber);
        return i;
      }
    }
    return -1;
  }

  void unsubscribe(size_t ix) {
    if (ix == -1)
      return;
    subscribers.at(ix).reset();
  }

  void broadcast(T t) {
    for (auto &sub : subscribers) {
      if (sub) {
        sub->post(t);
      }
    }
  }
};