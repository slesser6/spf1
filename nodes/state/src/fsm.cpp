#include "fsm.hpp"
#include <iostream>

FSM::FSM() : current_state_(State::INIT) {}

void FSM::setState(State new_state) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_state_ = new_state;
}

FSM::State FSM::getState() {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_;
}

std::string FSM::toString() {
  switch (current_state_) {
  case State::INIT:
    return "INIT";
  case State::IDLE:
    return "IDLE";
  case State::FIND:
    return "FIND";
  case State::NAV:
    return "NAV";
  case State::ALIGN:
    return "ALIGN";
  case State::CTRL:
    return "CTRL";
  case State::ERROR:
    return "ERROR";
  default:
    return "Unkown";
  }
}