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

#include <ecto/ecto.hpp>
#include <ecto/registry.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using ecto::tendrils;
namespace ecto_test
{
  namespace pt = boost::posix_time;
  namespace bp = boost::python;

  struct Throttle
  {
    unsigned period_usec;
    pt::ptime prevtime;
    ecto::spore<pt::ptime> in, out;

    static void declare_params(tendrils& parameters)
    {
      parameters.declare<double> ("rate", "Do not pass data more quickly than this many hz", 1.0);
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare<pt::ptime> ("in", "input");
      outputs.declare<pt::ptime> ("out", "output");
    }

    void configure(tendrils& parameters, tendrils& inputs, tendrils& outputs)
    {
      period_usec = 1e+06 / parameters.get<double>("rate");
      
      in = inputs.at("in");
      out = outputs.at("out");
      prevtime = pt::microsec_clock::universal_time() - pt::hours(24);
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      pt::ptime now(pt::microsec_clock::universal_time());
      // std::cout << "now: " << now << " prevtime : " <<  prevtime << "\n";
      pt::time_duration already_waited = now - prevtime;
      std::cout << this << " already waited usec = " << already_waited << "\n";
      int mustwait_usec = period_usec - already_waited.total_microseconds();

      if (mustwait_usec > 0)
        {
          std::cout << "Throttle usleep(" << mustwait_usec << ")\n";
          boost::this_thread::sleep(boost::posix_time::microseconds(mustwait_usec));

        }
      *out = *in;
      prevtime = pt::microsec_clock::universal_time();
      return 0;
    }
  };
}

ECTO_CELL(ecto_test, ecto_test::Throttle, "Throttle", "Throttle to a certain Hz frequency");

