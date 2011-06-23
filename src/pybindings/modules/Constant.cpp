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
  namespace bp = boost::python;

  struct Constant
  {
    spore<bp::object> value, out;

    static void declare_params(tendrils& params)
    {
      params.declare<bp::object>("value", "Value to output");
    }

    static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
    {
      out.declare<bp::object>("out", "out. box.");
    }

    void configure(tendrils& p, tendrils& i, tendrils& o)
    {
      value = p.at("value");
      out = o.at("out");
    }

    int process(const tendrils& i, tendrils& o)
    {
      std::cout << "value type=" << ((tendril::ptr)value)->type_name() << "\n";
      std::cout << "out type=" << ((tendril::ptr)out)->type_name() << "\n";
      tendril::ptr value_tp = value.tendril_ptr();
      tendril::ptr out_tp = out.tendril_ptr();
      out_tp->copy_value(*value_tp);
      std::cout << "Constant::process done." << std::endl;
      return ecto::OK;
    }
  };
}

ECTO_MODULE(ecto, ecto::Constant, "Constant", 
            "Constant node always outputs same value.");
