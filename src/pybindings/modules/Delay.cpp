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
#include <iostream>
#include <queue>

namespace ecto
{
  using ecto::tendrils;

  struct Delay
  {
    static void declare_params(tendrils& params)
    {
      params.declare<ecto::tendril::ptr>("value", "Value to delay");
    }

    static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      in["input"].reset(new ecto::tendril);
      out["output"].reset(new ecto::tendril);
      in.at("input")->set_doc("The input tendril, can assume any type.");
      out.at("output")->set_doc("The out tendril, can assume any type.");
      ecto::tendril::ptr value = parms.get<ecto::tendril::ptr>("value");
      if (value) // default value is none..
      {
        in.at("input")->copy_value(*value);
        out.at("output")->copy_value(*value);
      }
    }

    int process(const tendrils& in, tendrils& out)
    {
      delay_n_ = 1;
      buffer_.push(ecto::tendril());
      buffer_.back().copy_value(*in.at("input"));
      out.at("output")->copy_value(buffer_.front());
      if (buffer_.size() > delay_n_)
      {
        buffer_.pop();
      }
      return ecto::OK;
    }
    size_t delay_n_;
    std::queue<ecto::tendril> buffer_;
  };

}

ECTO_MODULE(ecto, ecto::Delay, "Delay", "Delay node.");
