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
#ifndef _BEAGLEG_STRUCTBUF_H
#define _BEAGLEG_STRUCTBUF_H

#include <stddef.h>
#include <functional>
#include <unistd.h>
#include <cstdint>

class StructbufReader {
public:
  // A function to read from some data source. Similar to read(2), it gets
  // a buffer and a maximums size it can write to and returns the number
  // of bytes it has actually written.
  // Returns a positive number for read bytes, zero if we are end-of-stream
  // and a negative number to indicate error.
  typedef std::function<ssize_t(uint8_t *buf, size_t size)> ReadFun;

  // The "buffer_size" determines the longest line we expect at maximum.
  // TODO(hzeller): right now, we don't gracefully deal with overlong lines.
  StructbufReader(size_t struct_size);
  ~StructbufReader();

  // Update content. It will be calling the ReadFun exactly once and updates
  // its internal buffer.
  // After this, you may call ReadLine() to extract as many lines as had
  // been waiting.
  // If you made sure that there is data available before calling Update(),
  // this will not block.
  int Update(ReadFun read_fun);

  // The tpical way this will be called: with a file descriptor that has some
  // bytes ready.
  int Update(int fd) {
    return Update([fd](uint8_t *buf, size_t len) {
            return read(fd, buf, len);
           });
  }

  // Return a current line if it is available. The line is a nul terminated
  // c-string.
  // If there is no current line pending, returns NULL.
  // It is a good idea to call this after a call to Update() in a loop until
  // you reach NULL to empty the buffer before the next Update() comes in.
  void* Read();

  // Currently stored in buffer.
  size_t size() const { return content_end_ - content_start_; }

private:
  const size_t struct_size_;
  const size_t len_;
  uint8_t *const buffer_start_;
  uint8_t *const buffer_end_;
  uint8_t *content_start_;
  uint8_t *content_end_;
};

class StructbufWriter {
public:
  // A function to read from some data source. Similar to read(2), it gets
  // a buffer and a maximums size it can write to and returns the number
  // of bytes it has actually written.
  // Returns a positive number for read bytes, zero if we are end-of-stream
  // and a negative number to indicate error.
  typedef std::function<ssize_t(uint8_t *buf, size_t size)> WriteFun;

  // The "buffer_size" determines the longest line we expect at maximum.
  // TODO(hzeller): right now, we don't gracefully deal with overlong lines.
  StructbufWriter(size_t struct_size);
  ~StructbufWriter();

  // Update content. It will be calling the ReadFun exactly once and updates
  // its internal buffer.
  // After this, you may call ReadLine() to extract as many lines as had
  // been waiting.
  // If you made sure that there is data available before calling Update(),
  // this will not block.
  int Update(WriteFun write_fun, void *s);

  // The tpical way this will be called: with a file descriptor that has some
  // bytes ready.
  int Update(int fd, void *s) {
    return Update([fd, s](uint8_t *buf, size_t len) {
            return write(fd, buf, len);
          }, s);
  }

  // Currently stored in buffer.
  size_t size() const { return content_end_ - content_start_; }

private:
  const size_t struct_size_;
  const size_t len_;
  uint8_t *const buffer_start_;
  uint8_t *const buffer_end_;
  uint8_t *content_start_;
  uint8_t *content_end_;
};

#endif  // _BEAGLEG_STRUCTBUF_H
