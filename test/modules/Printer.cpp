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

using ecto::tendrils;
namespace ecto_test
{
  struct Printer
  {
    struct PrintFunctions
    {
      template <typename T>
      static void declare(ecto::tendrils&inputs)
      {
        inputs.declare<T> ("in", "what to print");
      }

      template <typename T>
      static void process(const ecto::tendrils&inputs,ecto::tendrils&outputs)
      {
        std::cout << "***** " << inputs.get<T> ("in") << " ***** ";
      }

      std::map<std::string, boost::function<void(ecto::tendrils&inputs)> > declares;
      std::map<std::string, boost::function<void(const ecto::tendrils&inputs,ecto::tendrils&outputs)> > processes;
      PrintFunctions()
      {
        declares["int"] = &declare<int>;
        declares["double"] = &declare<double>;
        declares["string"] = &declare<std::string>;
        declares["bool"] = &declare<bool>;
        processes[ecto::name_of<int>()] = &process<int>;
        processes[ecto::name_of<double>()] = &process<double>;
        processes[ecto::name_of<std::string>()] = &process<std::string>;
        processes[ecto::name_of<bool>()] = &process<bool>;
      }
    };

    static PrintFunctions pfs;
    static void declare_params(tendrils& parameters)
    {
      parameters.declare<std::string>("print_type","The type string for what i'm to print... int, double, bool, string.")
          .set_default_val("double")
          ;
    }

    static void declare_io(const ecto::tendrils& parameters, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      std::string print_type = parameters.get<std::string>("print_type");
      pfs.declares[print_type](inputs);
    }

    int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      pfs.processes[inputs.at("in")->type_name()](inputs,outputs);
      std::cout << this << std::endl;
      return ecto::OK;
    }
  };

  Printer::PrintFunctions Printer::pfs;
}

ECTO_CELL(ecto_test, ecto_test::Printer, "Printer", 
            "A printer of int, double, string, bool. "
            "Use the print_type parameter to specify type.  Default is double");

