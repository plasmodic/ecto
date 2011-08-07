/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include <stdint.h>//int64_t
#define ECTO_LOG_STATS 1

#include <ecto/util.hpp>
#include <ecto/log.hpp>

namespace ecto {
  namespace profile {

    unsigned long read_tsc(void);

    struct ECTO_EXPORT stats_type
    {
      unsigned ncalls;
      int64_t total_ticks;
    };

    struct ECTO_EXPORT stats_collector
    {
      int64_t start;
      stats_type& stats;
      const std::string& instancename;

      stats_collector(const std::string& n, stats_type& stats) 
        : start(read_tsc()), stats(stats), instancename(n) 
      { 
        ++stats.ncalls;
        ECTO_LOG_PROCESS(instancename, start, 1);
      }

      ~stats_collector() {
        int64_t tsc = read_tsc();
        ECTO_LOG_PROCESS(instancename, tsc, 0);
        stats.total_ticks += (tsc - start);
      }
    };

    ECTO_EXPORT double elapsed_time(const stats_type& stats);
    ECTO_EXPORT double period(const stats_type& stats);
    ECTO_EXPORT double frequency(const stats_type& stats);
 }
}
