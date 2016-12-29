// -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-

#ifndef FD_TIMER_H_
#define FD_TIMER_H_

#include <functional>
#include <sys/timerfd.h>

#include "fd-mux.h"

// This needs a better name.
class FDTimer {
public:
    typedef std::function<void()> Callback;
    FDTimer(float t, FDMultiplexer *fmux, const Callback &callback)
            : callback_(callback) {
    timer_ = timerfd_create(CLOCK_REALTIME, 0);
    if (timer_ < 0) Log_error("timerfd_create");

    Log_debug("Created TIMER with fd: %d", timer_);
    struct itimerspec timeout = {0};
    float i, f;
    f = modff(t / 1e3, &i);
    timeout.it_value.tv_sec = (int) i;
    timeout.it_value.tv_nsec = (long) (f * 1e9l);

    if (timerfd_settime(timer_, 0, &timeout, NULL) == -1)
      Log_error("timerfd_settime");

    fmux->RunOnReadable(timer_, [this](){
      return Consume();
    });
    }

    ~FDTimer() { close(timer_); }

private:
    bool Consume() {
      Log_debug("Callbacking");
      callback_();
      delete this;
      return false;
    }
    int timer_;
    const Callback callback_;
};

#endif // FD_TIMER_H_
