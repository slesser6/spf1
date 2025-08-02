#ifndef FSM_HPP_
#define FSM_HPP_

#include <string>
#include <map>
#include <functional>
#include <mutex>

class FSM
{
public:
    enum class State
    {
        INIT,
        IDLE,
        NAV,
        ALIGN,
        CTRL,
        ERROR
    };
    FSM();

    void setState(State new_state);
    State getState();
    std::string toString();

private:
    State current_state_;
    std::mutex mutex_;
};

#endif