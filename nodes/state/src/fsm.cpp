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
    return "Initializing";
  case State::IDLE:
    return "Waiting";
  case State::NAV:
    return "Navigating";
  case State::ALIGN:
    return "Aligning";
  case State::CTRL:
    return "User control";
  case State::ERROR:
    return "Error";
  default:
    return "Unkown";
  }
}