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
#include <stdio.h>
#include <string.h>
#include <cstdint>
#include "struct-buffer.h"

StructbufReader::StructbufReader(size_t struct_size)
  : struct_size_(struct_size), len_(struct_size_ * 2),
    buffer_start_(new uint8_t [len_]), buffer_end_(buffer_start_ + len_),
    content_start_(buffer_start_), content_end_(buffer_start_) {
}
StructbufReader::~StructbufReader() { delete [] buffer_start_; }

int StructbufReader::Update(ReadFun read_fun) {
  if (content_start_ - buffer_start_ > (int)(len_ / 2)) {
    const size_t copy_len = size();
    memmove(buffer_start_, content_start_, copy_len);
    content_start_ = buffer_start_;
    content_end_ = buffer_start_ + copy_len;
  }
  ssize_t r = read_fun(content_end_, buffer_end_ - content_end_);
  // TODO(hzeller): if we get zero, we should consider this as end-of-stream
  // and potentially regard the buffer as 'complete' even if it doesn't have a
  // full line yet.
  if (r >= 0) content_end_ += r;   // so, what if r < 0 ?
  return r;
}

void* StructbufReader::Read() {
  if (content_end_ - content_start_ >= (int) struct_size_) {
    const uint8_t *s = content_start_;
    content_start_ += struct_size_;
    return (void *) s;
  }
  return NULL;
}

StructbufWriter::StructbufWriter(size_t struct_size)
  : struct_size_(struct_size), len_(struct_size_),
    buffer_start_(new uint8_t [len_]), buffer_end_(buffer_start_ + len_),
    content_start_(buffer_start_), content_end_(buffer_start_) {
}
StructbufWriter::~StructbufWriter() { delete [] buffer_start_; }

int StructbufWriter::Update(WriteFun write_fun, void *s) {
  // If there are still pieces from previous updates
  // Let's give them priority and discard s
  if (size() != 0) {
    ssize_t r = write_fun(content_start_, content_end_ - content_start_);
    content_start_ += r;
    return r;
  }
  content_start_ = buffer_start_;
  // Append to the current buffer the new struct to push
  // There's enough space
  memcpy(content_start_, s, struct_size_);
  content_end_ = content_start_ + struct_size_;

  ssize_t r = write_fun(content_start_, content_end_ - content_start_);
  if (r >= 0) content_start_ += r;   // so, what if r < 0 ?
  return r;
}
