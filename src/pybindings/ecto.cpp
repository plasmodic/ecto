#include <boost/python.hpp>
#include <ecto/ecto.hpp>

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
    void wrapSchedulers();
    void wrapStrand();
    void wrap_except();
    void wrap_ether();

    namespace {
      std::ofstream log_file;
      std::streambuf* stdout_orig = 0, *stderr_orig = 0, *log_rdbuf = 0;
    }
    void log_to_file(const std::string& fname)
    {
      log_file.close();
      log_rdbuf = 0;
      std::cout << "Redirecting C++ cout/cerr to '" << fname << "'\n";
      log_file.open(fname.c_str());
      stdout_orig = std::cout.rdbuf();
      stderr_orig = std::cerr.rdbuf();
      log_rdbuf = log_file.rdbuf();
      std::cout.rdbuf(log_rdbuf);
      std::cerr.rdbuf(log_rdbuf);
    }

    void unlog_to_file() {
      std::cout.flush();
      std::cerr.flush();
      log_file.close();
      assert(stdout_orig);
      assert(stderr_orig);
      std::cout.rdbuf(stdout_orig);
      std::cerr.rdbuf(stderr_orig);
      log_rdbuf = 0;
    }

  }
}

ECTO_INSTANTIATE_REGISTRY(ecto)
 
void dummy() { std::cout << __PRETTY_FUNCTION__ << "\n"; }

BOOST_PYTHON_MODULE(ecto)
{
  bp::class_<ecto::tendril::none>("no_value");

  ecto::py::wrapConnection();
  ecto::py::wrapPlasm();
  ecto::py::wrapModule();
  ecto::py::wrapTendrils();
  ecto::py::wrapSchedulers();
  ecto::py::wrapStrand();
  ecto::py::wrap_except();

  bp::def("hardware_concurrency", &boost::thread::hardware_concurrency);

  // for setting breakpoints
  bp::def("dummy", &dummy);

  // use this if you're embedding ipython and dont want to see
  // your cout/cerr
  bp::def("log_to_file", &ecto::py::log_to_file);
  bp::def("unlog_to_file", &ecto::py::unlog_to_file);
  ECTO_REGISTER(ecto);
  ecto::py::wrap_ether();
}

