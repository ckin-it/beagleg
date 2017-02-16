// -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-

#include "fd-mux.h"
#include "logging.h"

#include <vector>
#include <algorithm>
#include <sys/select.h>
#include <errno.h>
#include <string.h>

bool FDMultiplexer::RunOnReadable(int fd, const Handler &handler) {
  return r_handlers_.insert({ fd, handler }).second;
}

bool FDMultiplexer::IsRegisteredReadable(int fd) const {
    return r_handlers_.find(fd) != r_handlers_.end();
}

bool FDMultiplexer::RunOnWritable(int fd, const Handler &handler) {
  return w_handlers_.insert({ fd, handler }).second;
}

bool FDMultiplexer::IsRegisteredWritable(int fd) const {
    return w_handlers_.find(fd) != w_handlers_.end();
}

bool FDMultiplexer::RunOnException(int fd, const Handler &handler) {
  return e_handlers_.insert({ fd, handler }).second;
}

bool FDMultiplexer::IsRegisteredExceptioned(int fd) const {
    return e_handlers_.find(fd) != e_handlers_.end();
}

void FDMultiplexer::ScheduleDelete(int fd) {
  if (r_handlers_.find(fd) != r_handlers_.end()) {
    to_delete_r_.push_back(fd);
  } else if (w_handlers_.find(fd) != w_handlers_.end()) {
    to_delete_w_.push_back(fd);
  } else if (e_handlers_.find(fd) != e_handlers_.end()) {
    to_delete_e_.push_back(fd);
  }
}

void FDMultiplexer::Loop() {
    fd_set read_fds;
    fd_set write_fds;
    fd_set exception_fds;

    for (;;) {
        int maxfd = -1;
        FD_ZERO(&read_fds);
        FD_ZERO(&write_fds);
        FD_ZERO(&exception_fds);

        // Readers
        for (const auto &it : r_handlers_) {
            if (it.first >= maxfd) maxfd = it.first+1;
            FD_SET(it.first, &read_fds);
        }

        // Writers
        for (const auto &it : w_handlers_) {
            if (it.first >= maxfd) maxfd = it.first+1;
            FD_SET(it.first, &write_fds);
        }

        // Exceptions
        for (const auto &it : e_handlers_) {
            if (it.first >= maxfd) maxfd = it.first+1;
            FD_SET(it.first, &exception_fds);
        }

        if (maxfd < 0) {
            // file descriptors only can be registred from within handlers
            // or before running the Loop(). If no filedesctiptors are left,
            // there is no chance for any to re-appear, so we can exit.
            fprintf(stderr, "No filedescriptor registered. Exiting loop()");
            break;
        }

        int fds_ready = select(maxfd, &read_fds, &write_fds, &exception_fds, nullptr);
        if (fds_ready < 0) {
            perror("select() failed");
            break;
        }

        if (fds_ready == 0) {
            // Timeout situation. We are not registering timeout handlers
            // currently.
            continue;
        }

        // Handle reads
        for (const auto &it : r_handlers_) {
            if (FD_ISSET(it.first, &read_fds)) {
                const bool retrigger = it.second();
                if (!retrigger) {
                    to_delete_r_.push_back(it.first);
                }
                if (--fds_ready == 0)
                    break;
            }
        }
        for (int i : to_delete_r_) {
            r_handlers_.erase(i);
        }

        // Handle writes
        for (const auto &it : w_handlers_) {
            if (FD_ISSET(it.first, &write_fds)) {
                const bool retrigger = it.second();
                if (!retrigger) {
                    to_delete_w_.push_back(it.first);
                }
                if (--fds_ready == 0)
                    break;
            }
        }
        for (int i : to_delete_w_) {
            w_handlers_.erase(i);
        }

        // Handle exceptions
        for (const auto &it : e_handlers_) {
            if (FD_ISSET(it.first, &exception_fds)) {
                Log_debug("no fds");
                const bool retrigger = it.second();
                if (!retrigger) {
                    to_delete_e_.push_back(it.first);
                }
                if (--fds_ready == 0)
                    break;
            }
        }
        for (int i : to_delete_e_) {
            e_handlers_.erase(i);
        }

        to_delete_r_.clear();
        to_delete_w_.clear();
        to_delete_e_.clear();
    }
}
