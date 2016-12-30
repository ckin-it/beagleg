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

#include <grpc++/grpc++.h>
#include "control-interface.grpc.pb.h"

#include "grpc-control-server.h"
#include "gcode-machine-control.h"
#include "fd-mux.h"
#include "logging.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using beagleg::Empty;
using beagleg::MachineStats;
using beagleg::BeagleGControls;

// To add in the makefile
// protoc -I . --grpc_out=. --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` --cpp_out=. control-interface.proto

typedef enum commands {
  STOP,
  PAUSE,
  RESUME
} Command;

struct
class BeagleGControlsServiceImpl final : public BeagleGControls::Service {
public:
  BeagleGControlsServiceImpl(int pipe_fd[2]) {
    pipe_fd_[0] = pipe_fd[0]; // Reader
    pipe_fd_[1] = pipe_fd[1]; // Writer
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
    return Status::OK;
  }

private:
  void SendCommandToPipe(const Command command) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (write(pipe_fd_[1], &command, sizeof(Command)) < 0)
      perror("pip_fd write()");
  }

  std::mutex mutex_;
  int pipe_fd_;
};

class GrpcControlServer::Impl {
public:
  Impl(GCodeMachineControl *machine, FDMultiplexer *event_server);
  void Run();

private:
  GCodeMachineControl *const machine_;
  FDMultiplexer *const event_server_;

  int pipe_fd_[2];
  std::unique_ptr<std::thread> grpc_wait_;
  std::unique_ptr<Server> server_;
  void HandleCommand();
};

GrpcControlServer::Impl(GCodeMachineControl *machine,
                        FDMultiplexer *event_server)
                        : machine_(machine), event_server_(event_server) {
}

void GrpcControlServer::Impl::Run() {
  // Create the pipe()
  if(pipe(pipe_fd_) < 0) perror("pipe");

  std::string server_address("0.0.0.0:50051");
  BeagleGControlsServiceImpl service(pipe_fd_);

  ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(&service);
  // Finally assemble the server.
  server_(builder.BuildAndStart());
  Log_debug("Started the control server on %d", server_address);

  // Register the async task
  event_server_->RunOnReadable(pipe_fd_[0], [this](){ HandleCommand(); })

  // Init the server and launch the wait on a thread
  grpc_wait_(new std::thread([this](){ server->Wait(); });
}

void GrpcControlServer::Impl::HandleCommand() {
  Command command;
  if (read(pipe_fd_[0], &command, sizeof(command)) < 0)
    perror("HandleCommand pipe read()");

  switch (command) {
    case STOP: machine_->Stop(); break;
    case PAUSE: machine_->Pause(); break;
    case RESUME: machine_->Resume(); break;
  }

  return true;
}

void GrpcControlServer::GrpcControlServer::Run() {
  impl->Run();
}

GrpcControlServer::~GrpcControlServer(GCodeMachineControl *machine,
                                 FDMultiplexer *event_server) {
  delete impl_;
}

GrpcControlServer::GrpcControlServer(GCodeMachineControl *machine,
                                     FDMultiplexer *event_server)
  : impl_(new Impl(machine, event_server)) {
}
