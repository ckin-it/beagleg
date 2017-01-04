/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "motor-operations.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include <sys/timerfd.h>
#include <sys/time.h>
#include <unistd.h>

#include "motor-interface-constants.h"
#include "motion-queue.h"
#include "fd-timer.h"
#include "logging.h"


#define ENABLE_DELAY 500 // milli seconds

// We need two loops per motor step (edge up, edge down),
// So we need to multiply step-counts by 2
// This could be more, if we wanted to implement sub-step resolution with
// more than one bit output per step (probably only with hand-built drivers).
#define LOOPS_PER_STEP (1 << 1)

// If we do more than these number of steps, the fixed point fraction
// accumulate too much error.
#define MAX_STEPS_PER_SEGMENT (65535 / LOOPS_PER_STEP)

// TODO: don't store this singleton like, but keep in user_data of the MotorOperations
static float hardware_frequency_limit_ = 1e6;    // Don't go over 1 Mhz

static inline float sq(float x) { return x * x; }  // square a number
static inline double sqd(double x) { return x * x; }  // square a number
static inline int round2int(float x) { return (int) roundf(x); }

// Clip speed to maximum we can reach with hardware.
static float clip_hardware_frequency_limit(float v) {
  return v < hardware_frequency_limit_ ? v : hardware_frequency_limit_;
}

static float calcAccelerationCurveValueAt(int index, float acceleration) {
  // counter_freq * sqrt(2 / accleration)
  const float accel_factor = TIMER_FREQUENCY
    * (sqrtf(LOOPS_PER_STEP * 2.0f / acceleration)) / LOOPS_PER_STEP;
  // The approximation is pretty far off in the first step; adjust.
  const float c0 = (index == 0) ? accel_factor * 0.67605f : accel_factor;
  return c0 * (sqrtf(index + 1) - sqrtf(index));
}

#if 0
// Is acceleration in acceptable range ?
static char test_acceleration_ok(float acceleration) {
  if (acceleration <= 0)
    return 1;  // <= 0: always full speed.

  // Check that the fixed point acceleration parameter (that we shift
  // DELAY_CYCLE_SHIFT) fits into 32 bit.
  // Also 2 additional bits headroom because we need to shift it by 2 in the
  // division.
  const float start_accel_cycle_value = (1 << (DELAY_CYCLE_SHIFT + 2))
    * calcAccelerationCurveValueAt(0, acceleration);
  if (start_accel_cycle_value > 0xFFFFFFFF) {
    Log_error("Too slow acceleration to deal with. If really needed, "
              "reduce value of #define DELAY_CYCLE_SHIFT\n");
    return 0;
  }
  return 1;
}
#endif

void MotionQueueMotorOperations::EnqueueInternal(const LinearSegmentSteps &param,
                                                 int defining_axis_steps) {
  struct MotionSegment new_element = {};
  new_element.direction_bits = 0;

  // The defining_axis_steps is the number of steps of the axis that requires
  // the most number of steps. All the others are a fraction of the steps.
  //
  // The top bits have LOOPS_PER_STEP states (2 is the minium, as we need two
  // cycles for a 0 1 transition. So in that case we have 31 bit fraction
  // and 1 bit that overflows and toggles for the steps we want to generate.
  const uint64_t max_fraction = 0xFFFFFFFF / LOOPS_PER_STEP;
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    if (param.steps[i] < 0) {
      new_element.direction_bits |= (1 << i);
    }
    const uint64_t delta = abs(param.steps[i]);
    new_element.fractions[i] = delta * max_fraction / defining_axis_steps;
  }

  // TODO: clamp acceleration to be a minimum value.
  const int total_loops = LOOPS_PER_STEP * defining_axis_steps;
  // There are three cases: either we accelerate, travel or decelerate.
  if (param.v0 == param.v1) {
    // Travel
    new_element.loops_accel = new_element.loops_decel = 0;
    new_element.loops_travel = total_loops;
    const float travel_speed = clip_hardware_frequency_limit(param.v0);
    new_element.travel_delay_cycles = round2int(TIMER_FREQUENCY / (LOOPS_PER_STEP * travel_speed));
  } else if (param.v0 < param.v1) {
    // acclereate
    new_element.loops_travel = new_element.loops_decel = new_element.travel_delay_cycles = 0;
    new_element.loops_accel = total_loops;

    // v1 = v0 + a*t -> t = (v1 - v0)/a
    // s = a/2 * t^2 + v0 * t; subsitution t from above.
    // a = (v1^2-v0^2)/(2*s)
    float acceleration = (sq(param.v1) - sq(param.v0)) / (2.0f * defining_axis_steps);
    //fprintf(stderr, "M-OP HZ: defining=%d ; accel=%.2f\n", defining_axis_steps, acceleration);
    // If we accelerated from zero to our first speed, this is how many steps
    // we needed. We need to go this index into our taylor series.
    const int accel_loops_from_zero =
      round2int(LOOPS_PER_STEP * (sq(param.v0 - 0) / (2.0f * acceleration)));

    new_element.accel_series_index = accel_loops_from_zero;
    new_element.hires_accel_cycles =
      round2int((1 << DELAY_CYCLE_SHIFT) * calcAccelerationCurveValueAt(new_element.accel_series_index, acceleration));
  } else {  // v0 > v1
    // decelerate
    new_element.loops_travel = new_element.loops_accel = new_element.travel_delay_cycles = 0;
    new_element.loops_decel = total_loops;

    float acceleration = (sq(param.v0) - sq(param.v1)) / (2.0f * defining_axis_steps);
    //fprintf(stderr, "M-OP HZ: defining=%d ; decel=%.2f\n", defining_axis_steps, acceleration);
    // We are into the taylor sequence this value up and reduce from there.
    const int accel_loops_from_zero =
      round2int(LOOPS_PER_STEP * (sq(param.v0 - 0) / (2.0f * acceleration)));

    new_element.accel_series_index = accel_loops_from_zero;
    new_element.hires_accel_cycles =
      round2int((1 << DELAY_CYCLE_SHIFT) * calcAccelerationCurveValueAt(new_element.accel_series_index, acceleration));
  }

  new_element.aux = param.aux_bits;
  new_element.state = STATE_FILLED;
  // We were in stop, new stuff, so we are running again
  if (state_ == STOPPED) state_ = RUNNING;
  if (direct_) {
    if (!motor_enabled_) {
      backend_->MotorEnable(true);
      motor_enabled_ = true;
      usleep(ENABLE_DELAY * 1e3);
    }
    backend_->Enqueue(&new_element);
    return;
  }
  if (motor_enabled_) backend_->Enqueue(&new_element);
  else {
    backend_->MotorEnable(true);
    motor_enabled_ = true;
    // Force the motion queue to buffer all the segments
    backend_->ForceBufferized(true);
    backend_->Enqueue(&new_element);
    new FDTimer(ENABLE_DELAY, event_server_, [this]() {
      // Ok we can start for real now, disable the forced bufferization
      // And start the shoveler
      backend_->ForceBufferized(false);
    });
  }
}

static int get_defining_axis_steps(const LinearSegmentSteps &param) {
  int defining_axis_steps = abs(param.steps[0]);
  for (int i = 1; i < BEAGLEG_NUM_MOTORS; ++i) {
    if (abs(param.steps[i]) > defining_axis_steps) {
      defining_axis_steps = abs(param.steps[i]);
    }
  }
  return defining_axis_steps;
}

void MotionQueueMotorOperations::Enqueue(const LinearSegmentSteps &param) {
  const int defining_axis_steps = get_defining_axis_steps(param);

  if (defining_axis_steps == 0) {
    // No move, but we still have to set the bits.
    struct MotionSegment empty_element = {};
    empty_element.aux = param.aux_bits;
    empty_element.state = STATE_FILLED;
    backend_->Enqueue(&empty_element);
  }
  else if (defining_axis_steps > MAX_STEPS_PER_SEGMENT) {
    // We have more steps that we can enqueue in one chunk, so let's cut
    // it in pieces.
    const double a = (sqd(param.v1) - sqd(param.v0))/(2.0*defining_axis_steps);
    const int divisions = (defining_axis_steps / MAX_STEPS_PER_SEGMENT) + 1;
    int64_t hires_steps_per_div[BEAGLEG_NUM_MOTORS];
    for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
      // (+1 to fix rounding trouble in the LSB)
      hires_steps_per_div[i] = ((int64_t)param.steps[i] << 32)/divisions + 1;
    }

    struct LinearSegmentSteps previous = {}, accumulator = {}, output;
    int64_t hires_step_accumulator[BEAGLEG_NUM_MOTORS] = {0};
    double previous_speed = param.v0;   // speed calculation in double

    output.aux_bits = param.aux_bits;  // use the original Aux bits for all segments
    for (int d = 0; d < divisions; ++d) {
      for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
        hires_step_accumulator[i] += hires_steps_per_div[i];
        accumulator.steps[i] = hires_step_accumulator[i] >> 32;
        output.steps[i] = accumulator.steps[i] - previous.steps[i];
      }
      const int division_steps = get_defining_axis_steps(output);
      // These squared values can get huge, lets not loose precision
      // here and do calculations in double (otherwise our results can
      // be a little bit off and fail to reach zero properly).
      const double v0squared = sqd(previous_speed);
      // v1 = v0 + a*t; t = (sqrt(v0^2 + 2 * a * steps) - v0) / a
      // -> v1 = sqrt(v0^ + 2 * a * steps)
      const double v1squared = v0squared + 2.0 * a * division_steps;
      // Rounding errors can make v1squared slightly negative...
      const double v1 = v1squared > 0.0 ? sqrt(v1squared) : 0;
      output.v0 = previous_speed;
      output.v1 = v1;
      EnqueueInternal(output, division_steps);
      previous = accumulator;
      previous_speed = v1;
    }
  } else {
    EnqueueInternal(param, defining_axis_steps);
  }
}

void MotionQueueMotorOperations::MotorEnable(bool on) {
  if (direct_) {
    backend_->WaitQueueEmpty();
    backend_->MotorEnable(on);
    motor_enabled_ = on;
  } else {
    backend_->OnEmptyQueue([this, on](){
      backend_->MotorEnable(on);
      motor_enabled_ = on;
    });
  }
}

void MotionQueueMotorOperations::WaitQueueEmpty() {
  backend_->WaitQueueEmpty();
}

static float get_time(struct timeval s_time) {
  struct timeval t;
  gettimeofday(&t, 0);
  long elapsed = (t.tv_sec - s_time.tv_sec) * 1e6f + t.tv_usec - s_time.tv_usec;
  return elapsed / 1e6f;
}


class SpeedFactorProfiler {
public:
  ~SpeedFactorProfiler() {
    close(timer_);
  }

  SpeedFactorProfiler(FDMultiplexer *event_server,
                      State *state,
                      MotionQueue *backend,
                      State next_state,
                      long ns,
                      float et,
                      const Callback &callback)
                      : event_server_(event_server), state_(state),
                        backend_(backend), next_state_(next_state), ns_(ns),
                        et_(et), callback_(callback),
                        final_value_((next_state == RUNNING) ? 1 : 0),
                        sign_((next_state == RUNNING) ? 1 : -1),
                        quota_((next_state == RUNNING) ? 0 : 1) {
    assert(et_ > 0);
    // Start time
    gettimeofday(&st_, 0);
    // Create a timer
    timer_ = timerfd_create(CLOCK_REALTIME, 0);
    if (timer_ == -1) perror("timerfd_create");

    struct itimerspec timeout = {0};
    timeout.it_value.tv_nsec = ns_;

    if (timerfd_settime(timer_, 0, &timeout, NULL) == -1)
      perror("timerfd_settime");

    event_server_->RunOnReadable(timer_, [this](){
      // Consume the timer
      read(timer_, NULL, sizeof(uint64_t));
      return NextFactorValue();
    });
  }

  bool NextFactorValue() {
    const float ct = get_time(st_);
    if (ct > et_) { // We are done
      backend_->SetSpeedFactor(final_value_);
      if (next_state_ == PAUSED) {
        backend_->MotorEnable(false);
        // the state motor_enabled_ is already changed the parent function
      } else if (next_state_ == STOPPED ) {
        backend_->MotorEnable(false);
        // the state motor_enabled_ is already changed the parent function
        backend_->Reset();
      }
      *state_ = next_state_;
      callback_();
      delete this;
      return false;
    }

    const float factor = quota_ + sign_ * ct / et_;
    backend_->SetSpeedFactor(factor);

    // Arm it again
    struct itimerspec timeout = {0};
    timeout.it_value.tv_nsec = ns_;
    if (timerfd_settime(timer_, 0, &timeout, NULL) == -1)
      perror("timerfd_settime");
    return true;
  }

private:
  int timer_;
  struct timeval st_;

  FDMultiplexer *event_server_;
  State *const state_;
  MotionQueue *const backend_;
  const State next_state_;
  const long ns_;
  const float et_;
  const Callback callback_;
  const float final_value_;
  const float sign_;
  const float quota_;
};

void MotionQueueMotorOperations::RunAsyncStop(const Callback &callback) {
  if (state_ == STOPPED) return;
  else if (state_ == PAUSED) backend_->Reset();
  motor_enabled_ = false;
  new SpeedFactorProfiler(event_server_, &state_, backend_, STOPPED, 1e5, 1,
                          callback);
}

void MotionQueueMotorOperations::RunAsyncPause() {
  if (state_ == STOPPED || state_ == PAUSED) return;
  motor_enabled_ = false;
  new SpeedFactorProfiler(event_server_, &state_, backend_, PAUSED, 1e5, 1,
                          [](){});
}

void MotionQueueMotorOperations::RunAsyncResume() {
  if (state_ != PAUSED) return;
  // RESUME? Enable motors
  backend_->MotorEnable(true);
  motor_enabled_ = true;
  new FDTimer(ENABLE_DELAY, event_server_, [this]() {
    new SpeedFactorProfiler(event_server_, &state_, backend_, RUNNING, 1e5, 1,
                            [](){});
  });
}

void MotionQueueMotorOperations::GetRealtimeStatus(
    int pos_steps[BEAGLEG_NUM_MOTORS], unsigned short *aux_status) {
  MotorsRegister steps;
  backend_->GetMotorsStatus(&steps, aux_status);
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    // We are not actually retrieving loops, but loops * 2
    // that's due to the max_fraction in pru-motion-queue which is not
    // 0xFFFFFFFF / LOOPS_PER_STEP
    // pos_steps[i] = steps[i] / LOOPS_PER_STEP; // Convert into steps
    pos_steps[i] = steps[i];
  }
}

void MotionQueueMotorOperations::RunOnEmptyQueue(const Callback &callback) {
  backend_->OnEmptyQueue(callback);
}
