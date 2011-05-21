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

namespace hello_ecto
{

using ecto::tendrils;

struct Printer :  ecto::module_interface
{
  void initialize(tendrils& params)
  {
    params.declare<std::string> ("str", "The default string to print", "hello");
  }

  void configure(const tendrils& parms, tendrils& in, tendrils& out)
  {
    in.declare<std::string> ("str", "The string to print.", parms.get<std::string> ("str"));
  }

  void process(const tendrils& parms, const tendrils& in, tendrils& /*out*/)
  {
    std::cout << in.get<std::string> ("str") << std::endl;
  }
};

struct Reader : ecto::module_interface
{
  void configure(const tendrils& parms, tendrils& in, tendrils& out)
  {
    out.declare<std::string> ("output", "Output from standard in");
  }

  void process(const tendrils& parms, const tendrils& in, tendrils& out)
  {
    std::string o;
    std::cin >> o;
    out.get<std::string> ("output") = o;
  }
};

}

BOOST_PYTHON_MODULE(hello_ecto)
{
  using namespace hello_ecto;
  ecto::wrap<Printer>("Printer", "Prints a string input to standard output.");
  ecto::wrap<Reader>("Reader", "Reads input from standard input.");
}
