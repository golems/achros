/* -*- mode: C++; c-basic-offset: 2 -*- */
/* ex: set shiftwidth=2 tabstop=2 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "achros.hpp"
#include <cstring>
#include <ros/assert.h>


using namespace std;

namespace ros
{

TransportAch::TransportAch( const char *channel_name )
: is_open_(false), get_options_(0)
{
  chan_name_ = strdup(channel_name);
}

TransportAch::~TransportAch()
{
  if (is_open_) {
    this->close();
  }
}

int TransportAch::open()
{
  ROS_ASSERT(!is_open_);
  int r = ach_open(&chan_, chan_name_, NULL);
  if( ACH_OK == r ) {
    return ach_flush(&chan_);
  } else {
    return r;
  }
}

int TransportAch::create(size_t index_slots, size_t msg_bytes)
{
  return ach_create(chan_name_, index_slots, msg_bytes, NULL);
}

void TransportAch::close()
{
  ROS_ASSERT(is_open_);
  ach_close(&chan_); // FIXME: what to do if this fails?
}

int32_t TransportAch::read(uint8_t* buffer, uint32_t size) {
  // FIXME: how to handle timeouts?
  // FIXME: what to do with errors, ie ACH_MISSED_FRAME, ACH_STALE_FRAMES, ACH_OVERFLOW?
  size_t frame_size = 0;
  int r = ach_get( &chan_,
                   buffer, size, &frame_size,
                   NULL /*wait forever*/ ,
                   (ach_get_opts_t)get_options_ );
  if( ACH_OK == r ) {
    ROS_ASSERT(frame_size <= 0x7fffffff);
    return frame_size;
  } else {
    return -1;
  }
}

int32_t TransportAch::write(uint8_t* buffer, uint32_t size) {
  // FIXME: what to do with errors?
  int r = ach_put(&chan_, buffer, size);
  if( ACH_OK == r ) {
    return size;
  } else {
    return -1;
  }
}

std::string TransportAch::getTransportInfo() {
  return "ACHROS channel " + string(chan_name_);
}



}
