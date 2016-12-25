/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2016 Henner Zeller <h.zeller@acm.org>,
 *          Leonardo Romor <leonardo.romor@gmail.com>,
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
#ifndef BEAGLEG_PRU_HARDWARE_INTERFACE_
#define BEAGLEG_PRU_HARDWARE_INTERFACE_

#include <cstddef>

// Pru hardware controls
class PruHardwareInterface {
public:
  virtual ~PruHardwareInterface() {}

  // Initialize the hardware, interrupts, etc...
  // return value: returns if the operation was successful.
  virtual bool Init() = 0;

  // Returns the file descriptor associated with a pru interrupt
  virtual int EventFd() = 0;

  // Retrieve the pointer of the pru mapping and initialize the memory.
  virtual bool AllocateSharedMem(void **pru_mmap, const size_t size) = 0;

  // Enable the PRU and start predetermined program.
  virtual bool StartExecution() = 0;

  // Wait for a beagleg-mapped event. Return number of events that have occured.
  virtual unsigned WaitEvent() = 0;

  // Halt the PRU
  virtual bool Shutdown() = 0;

  // Reset back the PRU
  virtual void ResetPru() = 0;
};

class UioPrussInterface : public PruHardwareInterface {
public:
  bool Init();
  bool AllocateSharedMem(void **pru_mmap, const size_t size);
  int EventFd();
  bool StartExecution();
  unsigned WaitEvent();
  bool Shutdown();
  void ResetPru();
};

#endif  // BEAGLEG_PRU_HARDWARE_INTERFACE_
