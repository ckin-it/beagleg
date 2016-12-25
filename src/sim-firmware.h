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

#include "motion-queue.h"

#include <stdio.h>

class SimFirmwareQueue : public MotionQueue {
public:
  SimFirmwareQueue(FILE *out, int relevant_motors = MOTION_MOTOR_COUNT);
  virtual ~SimFirmwareQueue();

  virtual void Enqueue(MotionSegment *segment);
  virtual bool IsQueueEmpty() { return true; }
  virtual void WaitQueueEmpty() {}
  virtual void MotorEnable(bool on) {}
  virtual void Shutdown(bool flush_queue) {}
  virtual void GetMotorsLoops(MotorsRegister *absolute_pos_loops) {}
  // Set the speed factor, > 1 faster, == 1 normal speed, == 0 stop.
  virtual void SetSpeedFactor(const float factor) {}
  virtual void Reset() {};
  virtual int EventFd() { return -1; };

private:
  class Averager;

  FILE *const out_;
  const int relevant_motors_;
  Averager *const averager_;
};
