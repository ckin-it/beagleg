/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2016 Henner Zeller <h.zeller@acm.org>
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
#ifndef GRPC_CONTROL_SERVER_H_
#define GRPC_CONTROL_SERVER_H_

#include "gcode-machine-control.h"
#include "fd-mux.h"

class GrpcControlServer {
public:

  GrpcControlServer(GCodeMachineControl *machine,
                                 FDMultiplexer *event_server);
  ~GrpcControlServer();

  void Run();

private:

  class Impl;
  Impl *const impl_;  // opaque state.
};

#endif // GRPC_CONTROL_SERVER_H_
