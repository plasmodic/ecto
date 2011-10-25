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

#include <ecto/forward.hpp>
#include <ecto/util.hpp>
#include <ecto/log.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace ecto {
  namespace profile {

    unsigned long read_tsc(void);

    struct graph_stats_type
    {
      boost::posix_time::ptime start_time, stop_time;
      boost::posix_time::time_duration cumulative_time;
      unsigned long start_tick, stop_tick, cumulative_ticks;
      graph_stats_type();
      void start();
      void stop();
      std::string as_string(graph::graph_t& g);
    };

    struct graphstats_collector
    {
      graph_stats_type& gs_;
      graphstats_collector(graph_stats_type& gs)
        : gs_(gs)
      {
        gs_.start();
      }

      ~graphstats_collector()
      {
        gs_.stop();
      }
    };

    struct ECTO_EXPORT stats_type
    {
      stats_type();
      unsigned ncalls;
      int64_t total_ticks;
      bool on;

      double elapsed_time();
      double frequency();
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
        stats.on = true;
        //ECTO_LOG_PROCESS(instancename, start, stats.ncalls, 1);
      }

      ~stats_collector() {
        int64_t tsc = read_tsc();
        //ECTO_LOG_PROCESS(instancename, tsc, stats.ncalls, 0);
        stats.total_ticks += (tsc - start);
        stats.on = false;
      }
    };

 }
}
