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


#ifndef ACHROS_HPP
#define ACHROS_HPP

/** \file achros.cpp
 */

#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <ach.h>
#include <ros/types.h>
#include <ros/transport/transport.h>

namespace ros
{

class TransportAch;
typedef boost::shared_ptr<TransportAch> TransportAchPtr;

/**
 * \brief AchROS transport
 */
class /*ROSCPP_DECL*/ TransportAch /*: public Transport*/
{
public:
  TransportAch( const char *channel_name );
  virtual ~TransportAch();

  /** \brief Create the channel */
  int create(size_t index_slots=16, size_t msg_bytes=8192);
  /** \brief Open the channel */
  int open();
  /** \brief Option flags for ach_get */
  void setGetOptions( int opts ) { get_options_ = opts; }

  // overrides from Transport
  virtual int32_t read(uint8_t* buffer, uint32_t size);
  virtual int32_t write(uint8_t* buffer, uint32_t size);

  // these don't mean anything to Ach
  virtual void enableWrite() {}
  virtual void disableWrite() {}
  virtual void enableRead() {}
  virtual void disableRead() {}

  virtual void close();

  virtual std::string getTransportInfo();

  virtual void parseHeader(const Header& header);

  virtual const char* getType() { return "ACHROS"; }

protected:
  ach_channel_t chan_;
  const char *chan_name_;
  bool is_open_;
  int get_options_;
};


}


#endif //ACHROS_HPP
