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

// Implementation of the MotionQueue interfacing the PRU.
// We are using some shared memory between CPU and PRU to communicate.

#include "motion-queue.h"

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <strings.h>
#include <stdlib.h>
#include <unistd.h>

#include "generic-gpio.h"
#include "fd-mux.h"
#include "pwm-timer.h"
#include "logging.h"
#include "hardware-mapping.h"
#include "pru-hardware-interface.h"

//#define DEBUG_QUEUE

// The communication with the PRU. We memory map the static RAM in the PRU
// and write stuff into it from here. Mostly this is a ring-buffer with
// commands to execute, but also configuration data, such as what to do when
// an endswitch fires.
struct PRUCommunication {
  volatile struct QueueStatus status;
  volatile struct MotionSegment ring_buffer[QUEUE_LEN];
  volatile uint32_t time_factor;
} __attribute__((packed));

#ifdef DEBUG_QUEUE
static void DumpMotionSegment(volatile const struct MotionSegment *e,
                              volatile struct PRUCommunication *pru_data) {
  if (e->state == STATE_EXIT) {
    Log_debug("enqueue[%02td]: EXIT", e - pru_data->ring_buffer);
  } else {
    MotionSegment copy = (MotionSegment&) *e;
    std::string line;
    line = StringPrintf("enqueue[%02td]: dir:0x%02x s:(%5d + %5d + %5d) = %5d ",
                        e - pru_data->ring_buffer, copy.direction_bits,
                        copy.loops_accel, copy.loops_travel, copy.loops_decel,
                        copy.loops_accel + copy.loops_travel + copy.loops_decel);

    if (copy.hires_accel_cycles > 0) {
      line += StringPrintf("accel : %5.0fHz (%d loops);",
                           TIMER_FREQUENCY /
                           (2.0*(copy.hires_accel_cycles >> DELAY_CYCLE_SHIFT)),
                           copy.hires_accel_cycles >> DELAY_CYCLE_SHIFT);
    }
    if (copy.travel_delay_cycles > 0) {
      line += StringPrintf("travel: %5.0fHz (%d loops);",
                           TIMER_FREQUENCY / (2.0*copy.travel_delay_cycles),
                           copy.travel_delay_cycles);
    }
#if 0
    // The fractional parts.
    for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
      if (copy.fractions[i] == 0) continue;  // not interesting.
      line += StringPrintf("f%d:0x%08x ", i, copy.fractions[i]);
    }
#endif
    Log_debug("%s", line.c_str());
  }
}
#endif

// Store the required informations needed to backtrack the absolute position.
struct HistorySegment {
  HistorySegment() : fractions(), cumulative_loops(), direction_bits(0) {}
  uint32_t fractions[MOTION_MOTOR_COUNT];
  uint32_t cumulative_loops[MOTION_MOTOR_COUNT];
  uint8_t direction_bits;
  uint16_t aux;
};

// TODO: Avoid leakeage of context,
// in RegisterHistorySegment const uint64_t max_fraction = 0xFFFFFFFF;
// needs to be divided by LOOP_PER_STEP but this is something to be done
// inside motor operations
void PRUMotionQueue::RegisterHistorySegment(const MotionSegment &element) {
  const uint64_t max_fraction = 0xFFFFFFFF;
  const unsigned int last_insert_index = (queue_pos_ - 1) % QUEUE_LEN;
  const struct HistorySegment &previous
    = shadow_queue_[(last_insert_index - 1) % QUEUE_LEN];

  const uint64_t total_loops
    = element.loops_accel + element.loops_travel + element.loops_decel;
  const uint8_t direction_bits = element.direction_bits;

  HistorySegment *new_slot = &shadow_queue_[last_insert_index];

  // TODO: Motor operations already holds this information, we should wire it
  // until here in order to avoid this redundant operation.
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    const uint64_t fraction = element.fractions[i];
    new_slot->fractions[i] = fraction;
    const int sign = (direction_bits >> i) & 1 ? -1 : 1;
    uint64_t loops = fraction * total_loops;
    loops /= max_fraction;
    new_slot->cumulative_loops[i] = previous.cumulative_loops[i]
                                    + sign * loops;
  }

  new_slot->direction_bits = element.direction_bits;
  new_slot->aux = element.aux;
}

// TODO: GetMotorsLoops returns Auxes? hmm not very clear
void PRUMotionQueue::GetMotorsStatus(
    MotorsRegister *absolute_pos_loops, unsigned short *aux) {
  const uint64_t max_fraction = 0xFFFFFFFF;
  const struct QueueStatus status = *(struct QueueStatus*) &pru_data_->status;
  const struct HistorySegment &current = shadow_queue_[status.index];
  const uint64_t counter = status.counter;
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    const int64_t sign = (current.direction_bits >> i) & 1 ? -1 : 1;
    uint64_t loops = current.fractions[i];
    loops *= counter;
    loops /= max_fraction;
    (*absolute_pos_loops)[i] = current.cumulative_loops[i] - sign * loops;
  }
  if (aux != NULL) *aux = current.aux;
}

// Stop gap for compiler attempting to be overly clever when copying between
// host and PRU memory.
static void unaligned_memcpy(volatile void *dest, const void *src, size_t size) {
  volatile char *d = (volatile char*) dest;
  const char *s = (char*) src;
  const volatile char *end = d + size;
  while (d < end) {
    *d++ = *s++;
  }
}

void PRUMotionQueue::EnqueueInPru(MotionSegment *element) {
  const uint8_t state_to_send = element->state;
  assert(state_to_send != STATE_EMPTY);  // forgot to set proper state ?
  // Initially, we copy everything with 'STATE_EMPTY', then flip the state
  // to avoid a race condition while copying.
  element->state = STATE_EMPTY;

  volatile MotionSegment *queue_element = &pru_data_->ring_buffer[queue_pos_++];
  unaligned_memcpy(queue_element, element, sizeof(*queue_element));

  queue_pos_ %= QUEUE_LEN;

  // Fully initialized. Tell busy-waiting PRU by flipping the state.
  queue_element->state = state_to_send;

  // Register the last inserted motion segment in the shadow queue.
  RegisterHistorySegment(*element);
  #ifdef DEBUG_QUEUE
    DumpMotionSegment(queue_element, pru_data_);
  #endif
}

void PRUMotionQueue::Shovel() {
  // We got the interrupt, let's push all the segments as much as we can
  while (!overflow_queue_.empty() &&
         pru_data_->ring_buffer[queue_pos_].state == STATE_EMPTY) {
    EnqueueInPru(&overflow_queue_.front());
    overflow_queue_.pop();
  }
  if (overflow_queue_.empty()) {
    overflow_ = false;
  }
}

void PRUMotionQueue::Enqueue(MotionSegment *element) {
  if (overflow_) overflow_queue_.push(*element);
  else if (pru_data_->ring_buffer[queue_pos_].state != STATE_EMPTY) {
    // Queue full, push it in the overflow queue, trigger the shovel.
    overflow_queue_.push(*element);
    overflow_ = true;
    WakeUpEventHandler();
  } else {
    EnqueueInPru(element);
  }
}

void PRUMotionQueue::ForceBufferized(const bool status) {
  if (status) {
    overflow_ = true;
  } else {
    WakeUpEventHandler();
    EnqueueInPru(&overflow_queue_.front());
    overflow_queue_.pop();
  }
}

void PRUMotionQueue::OnEmptyQueue(const std::function<void()> &callback) {
  Log_debug("IS QUEUE EMPTY? %d", IsQueueEmpty());
  if (IsQueueEmpty()) { return callback(); }
  on_empty_queue_.push_back(callback);
  WakeUpEventHandler();
}

void PRUMotionQueue::WakeUpEventHandler() {
  if (handler_is_running_) return;
  fmux_->RunOnReadable(pru_interface_->EventFd(), [this](){
    return EventHandler();
  });
  handler_is_running_ = true;
}

bool PRUMotionQueue::EventHandler() {
  int i;
  read(pru_interface_->EventFd(), &i, 4);
  pru_interface_->ClearEvent();
  if (overflow_) Shovel();
  if (on_empty_queue_.size() > 0 && IsQueueEmpty()){
    for (const auto &callback : on_empty_queue_) { callback();
      on_empty_queue_.clear();
    }
  }
  if (!overflow_ && on_empty_queue_.size() == 0) { // Sleep
    handler_is_running_ = false;
    return false;
  }
  return true;
}

// Useful for async
bool PRUMotionQueue::IsQueueEmpty() {
  if (overflow_) return false; // It must be full
  const unsigned int last_insert_index = (queue_pos_ - 1) % QUEUE_LEN;
  return pru_data_->ring_buffer[last_insert_index].state == STATE_EMPTY;
}

void PRUMotionQueue::WaitQueueEmpty() {
  const unsigned int last_insert_index = (queue_pos_ - 1) % QUEUE_LEN;
  while (overflow_ ||
         pru_data_->ring_buffer[last_insert_index].state != STATE_EMPTY) {
    pru_interface_->WaitEvent();
  }
}

void PRUMotionQueue::SetSpeedFactor(const float factor) {
  assert(factor >= 0);
  // store factor * (1 << 16) in the pru
  if (factor == 0) {
    pru_data_->time_factor = 0xffffffff; // Pause
    return;
  }
  pru_data_->time_factor = (1 / factor) * (1 << 16);
}

static void clear_std_queue(std::queue<struct MotionSegment> &q) {
   std::queue<struct MotionSegment> empty;
   std::swap(q, empty);
}

void PRUMotionQueue::Reset() {
  // Better assert that motors are off or that the speed factor is 0
  for (int i = 0; i < QUEUE_LEN; ++i) {
    pru_data_->ring_buffer[i].state = STATE_EMPTY;
  }

  // The previous one in this case will be the last of the queue.
  // The pru will reset on index 0
  struct HistorySegment &previous = shadow_queue_[QUEUE_LEN -1];

  // Let's copy the actual number of loops to the *previous*
  // End
  const uint64_t max_fraction = 0xFFFFFFFF;
  const struct QueueStatus status = *(struct QueueStatus*) &pru_data_->status;
  const struct HistorySegment &current = shadow_queue_[status.index];
  const uint64_t counter = status.counter;

  // Update QUEUE_LEN - 1 containing the cumulative loops of now
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    const int64_t sign = (current.direction_bits >> i) & 1 ? -1 : 1;
    uint64_t loops = current.fractions[i];
    loops *= counter;
    loops /= max_fraction;
    previous.cumulative_loops[i] = current.cumulative_loops[i] - sign * loops;
  }

  // So that if we trigger GetMotorsStatus we read from the previous slot
  pru_data_->status.counter = 0;
  pru_data_->status.index = QUEUE_LEN - 1;
  // When we restart, the new enqueued slot will pick the cumulative loops
  // from the QUEUE_LEN - 1
  queue_pos_ = 0;
  // Clean the overflow queue
  clear_std_queue(overflow_queue_);
  on_empty_queue_.clear();
  overflow_ = false;
  handler_is_running_ = false;
  fmux_->ScheduleDelete(pru_interface_->EventFd());
  pru_interface_->ResetPru();
  SetSpeedFactor(1);
}

void PRUMotionQueue::MotorEnable(bool on) {
  hardware_mapping_->EnableMotors(on);
}

void PRUMotionQueue::Shutdown(bool flush_queue) {
  if (flush_queue) {
    struct MotionSegment end_element = {};
    end_element.state = STATE_EXIT;
    Enqueue(&end_element);
    WaitQueueEmpty();
  }
  pru_interface_->Shutdown();
  MotorEnable(false);
}

PRUMotionQueue::~PRUMotionQueue() { delete [] shadow_queue_; }

PRUMotionQueue::PRUMotionQueue(HardwareMapping *hw, PruHardwareInterface *pru,
                               FDMultiplexer *fmux)
                               : hardware_mapping_(hw),
                                 pru_interface_(pru),
                                 shadow_queue_(new HistorySegment[QUEUE_LEN]),
                                 fmux_(fmux),
                                 overflow_(false),
                                 handler_is_running_(false),
                                 overflow_queue_() {
  const bool success = Init();
  // For now, we just assert-fail here, if things fail.
  // Typically hardware-doomed event anyway.
  assert(success);
}

bool PRUMotionQueue::Init() {
  MotorEnable(false);  // motors off initially.
  if (!pru_interface_->Init())
    return false;

  if (!pru_interface_->AllocateSharedMem((void **) &pru_data_,
                                         sizeof(*pru_data_)))
    return false;

  for (int i = 0; i < QUEUE_LEN; ++i) {
    pru_data_->ring_buffer[i].state = STATE_EMPTY;
  }
  queue_pos_ = 0;
  pru_data_->time_factor = 0x00010000;

  return pru_interface_->StartExecution();
}
