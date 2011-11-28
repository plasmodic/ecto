// 
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 
#include <boost/python.hpp>
#include <ecto/ecto.hpp>
#include <ecto/registry.hpp>

#include <boost/thread.hpp>

#include <fstream>
namespace bp = boost::python;

//forward declare all modules.
namespace ecto {
  namespace py {
    void wrapTendrils();
    void wrapConnection();
    void wrapPlasm();
    void wrapModule();
    void wrapRegistry();
    void wrapSchedulers();
    void wrapStrand();
    void wrap_except();
    void wrap_ether();
    void wrap_black_box();
    void wrap_python_streambuf();
    void wrap_parameters();
    namespace {
      std::ofstream log_file;
      std::streambuf* stdout_orig = 0, *stderr_orig = 0, *log_rdbuf = 0;
    }

    void unlog_to_file() {
      std::cout.flush();
      std::cerr.flush();
      log_file.close();
      assert(stdout_orig); //fixme this is bad!
      assert(stderr_orig);
      std::cout.rdbuf(stdout_orig);
      std::cerr.rdbuf(stderr_orig);
      log_rdbuf = 0;
    }

    void log_to_file(const std::string& fname)
    {
      std::cout.flush();
      std::cerr.flush();
      log_file.close();
      std::cout << "Redirecting C++ cout to '" << fname << "'\n";
      log_file.open(fname.c_str());
      stdout_orig = std::cout.rdbuf();
      stderr_orig = std::cerr.rdbuf();
      log_rdbuf = log_file.rdbuf();
      std::cout.rdbuf(log_rdbuf);
      std::cerr.rdbuf(log_rdbuf);
      std::ostream os(stdout_orig);
      os << "Redirected.\n";
    }

  }
}

ECTO_INSTANTIATE_REGISTRY(ecto)
 
namespace ecto {
  namespace py {
    std::string versionstr() { return ECTO_VERSION_STRING; }
    unsigned abinum() { return ECTO_ABI_VERSION; }
    bp::tuple sonametuple() { return bp::make_tuple(ECTO_MAJOR_VERSION, ECTO_MINOR_VERSION, ECTO_PATCH_VERSION); }
  }
}

BOOST_PYTHON_MODULE(ecto)
{
  bp::class_<ecto::tendril::none>("no_value");

  ecto::py::wrapConnection();
  ecto::py::wrapPlasm();
  ecto::py::wrapModule();
  ecto::py::wrapRegistry();
  ecto::py::wrapTendrils();
  ecto::py::wrapSchedulers();
  ecto::py::wrapStrand();
  ecto::py::wrap_except();
  ecto::py::wrap_ether();
  ecto::py::wrap_black_box();
  ecto::py::wrap_python_streambuf();
  ecto::py::wrap_parameters();

  bp::def("hardware_concurrency", &boost::thread::hardware_concurrency);

  bp::def("version",&ecto::py::versionstr);
  bp::def("abi",&ecto::py::abinum);
  bp::def("soname",&ecto::py::sonametuple);

  // use this if you're embedding ipython and dont want to see
  // your cout/cerr
  bp::def("log_to_file", &ecto::py::log_to_file);
  bp::def("unlog_to_file", &ecto::py::unlog_to_file);
  ECTO_REGISTER(ecto);
}


