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
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>
#include <thread>
#include <sys/timerfd.h>
#include <stdint.h>

#include <grpc++/grpc++.h>
#include "control-interface.grpc.pb.h"

#include "gcode-machine-control.h"
#include "gcode-parser.h"
#include "grpc-control-server.h"
#include "fd-mux.h"
#include "logging.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerWriter;
using grpc::Status;
using beagleg::Empty;
using beagleg::MachineStats;
using beagleg::BeagleGControls;

// To add in the makefile
// protoc -I . --grpc_out=. --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` --cpp_out=. control-interface.proto

typedef struct {
  float axis_pos[GCODE_NUM_AXES];
  uint16_t aux;
} DataStream;

typedef enum commands {
  STOP,
  PAUSE,
  RESUME
} Command;

class BeagleGControlsServiceImpl final : public BeagleGControls::Service {
public:
  BeagleGControlsServiceImpl(int command_pipe_fd[2], int stream_pipe_fd[2]) {
    command_pipe_fd_[0] = command_pipe_fd[0]; // Reader
    command_pipe_fd_[1] = command_pipe_fd[1]; // Writer

    stream_pipe_fd_[0] = stream_pipe_fd[0]; // Reader
    stream_pipe_fd_[1] = stream_pipe_fd[1]; // Writer
  }

  Status Stop(ServerContext *context, const Empty *request, Empty *reply)
              override {
    // Call stop
    SendCommandToPipe(STOP);
    return Status::OK;
  }

  Status Pause(ServerContext *context, const Empty *request, Empty *reply)
               override {
    // Call pause
    SendCommandToPipe(PAUSE);
    return Status::OK;
  }

  Status Resume(ServerContext *context, const Empty *request, Empty *reply)
                override {
    // Call resume
    SendCommandToPipe(RESUME);
    return Status::OK;
  }

  Status MachineStatus(ServerContext *context, const Empty *request,
                       ServerWriter<MachineStats> *reply) override {
    // Get the position
    DataStream data;
    MachineStats payload;
    auto map = payload.mutable_axes();
    std::string name;
    for (;;) {
      if (read(stream_pipe_fd_[0], &data, sizeof(data)) < 0)
        perror("stream read pipe()");

      name = 'X'; (*map)[name] = data.axis_pos[0];
      name = 'Y'; (*map)[name] = data.axis_pos[1];
      name = 'Z'; (*map)[name] = data.axis_pos[2];
      name = 'E'; (*map)[name] = data.axis_pos[3];
      name = 'A'; (*map)[name] = data.axis_pos[4];
      name = 'B'; (*map)[name] = data.axis_pos[5];
      name = 'C'; (*map)[name] = data.axis_pos[6];
      name = 'U'; (*map)[name] = data.axis_pos[7];
      name = 'V'; (*map)[name] = data.axis_pos[8];
      name = 'W'; (*map)[name] = data.axis_pos[9];

      payload.set_auxes(data.aux);

      reply->Write(payload);
    }
    return Status::OK;
  }

private:
  void SendCommandToPipe(const Command command) {
    std::lock_guard<std::mutex> guard(write_mutex_);
    if (write(command_pipe_fd_[1], &command, sizeof(Command)) < 0)
      perror("pip_fd write()");
  }

  std::mutex write_mutex_;

  int command_pipe_fd_[2];
  int stream_pipe_fd_[2];

};

class GrpcControlServer::Impl {
public:
  Impl(GCodeMachineControl *machine, FDMultiplexer *event_server)
       : machine_(machine), event_server_(event_server) {}
  void Run();

private:
  GCodeMachineControl *const machine_;
  FDMultiplexer *const event_server_;

  int command_pipe_fd_[2];
  int stream_pipe_fd_[2];
  int timer_;
  std::thread *grpc_wait_;
  std::unique_ptr<Server> server_;
  bool HandleCommand();
  bool StatusStreamer();
};

// TODO: we don't really need two pipes, we could use just one
// and be careful of not triggering the other async task
void GrpcControlServer::Impl::Run() {
  // Create the pipe()
  if(pipe(command_pipe_fd_) < 0) perror("pipe");
  if(pipe(stream_pipe_fd_) < 0) perror("pipe");

  // Register the async task
  event_server_->RunOnReadable(command_pipe_fd_[0],
                               [this](){ return HandleCommand();});

  // Init the server and launch the wait on a thread
  grpc_wait_ = new std::thread([this](){
    std::string server_address("0.0.0.0:50051");
    BeagleGControlsServiceImpl service(command_pipe_fd_, stream_pipe_fd_);

    ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(&service);
    // Finally assemble the server.
    server_ = builder.BuildAndStart();
    Log_debug("Started the control server on %s", server_address.c_str());

    server_->Wait();
  });

  timer_ = timerfd_create(CLOCK_REALTIME, 0);
  if (timer_ == -1) perror("timerfd_create");

  struct itimerspec timeout = {0};
  timeout.it_value.tv_nsec = 1e8;

  if (timerfd_settime(timer_, 0, &timeout, NULL) == -1)
    perror("timerfd_settime");

  event_server_->RunOnReadable(timer_, [this](){
    return StatusStreamer();
  });
}

bool GrpcControlServer::Impl::StatusStreamer() {
  AxesRegister axes_pos;
  static DataStream data;
  machine_->GetRealtimeStatus(&axes_pos, &data.aux);


  for (const GCodeParserAxis axis : AllAxes()) {
    data.axis_pos[axis] = axes_pos[axis];
  }

  if (write(stream_pipe_fd_[1], &data, sizeof(data)) < 0)
    perror("pipe write()");

  struct itimerspec timeout = {0};
  timeout.it_value.tv_nsec = 1e8;

  if (timerfd_settime(timer_, 0, &timeout, NULL) == -1)
    perror("timerfd_settime");

  return true;
}

bool GrpcControlServer::Impl::HandleCommand() {
  Command command;
  if (read(command_pipe_fd_[0], &command, sizeof(command)) < 0)
    perror("HandleCommand pipe read()");

  switch (command) {
    case STOP: machine_->Stop(); break;
    case PAUSE: machine_->Pause(); break;
    case RESUME: machine_->Resume(); break;
  }

  return true;
}

void GrpcControlServer::GrpcControlServer::Run() {
  impl_->Run();
}

GrpcControlServer::~GrpcControlServer() { delete impl_; }

GrpcControlServer::GrpcControlServer(GCodeMachineControl *machine,
                                     FDMultiplexer *event_server)
  : impl_(new Impl(machine, event_server)) {
}
