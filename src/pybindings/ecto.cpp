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
    void wrapDelay();
    void wrap_except();
    void wrap_ether();
    void wrap_dealer();

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
      std::cout << "Redirecting C++ cout/cerr to '" << fname << "'\n";
      log_file.open(fname.c_str());
      stdout_orig = std::cout.rdbuf();
      stderr_orig = std::cerr.rdbuf();
      log_rdbuf = log_file.rdbuf();
      std::cout.rdbuf(log_rdbuf);
      std::cerr.rdbuf(log_rdbuf);
    }

  }
}

ECTO_INSTANTIATE_REGISTRY(ecto)
 
namespace
{
void dummy() { std::cout << __PRETTY_FUNCTION__ << "\n"; }
std::string versionstr(){return ECTO_VERSION_STRING;}
unsigned abinum(){return ECTO_ABI_VERSION;}
bp::tuple sonametuple(){return bp::make_tuple(ECTO_MAJOR_VERSION, ECTO_MINOR_VERSION, ECTO_PATCH_VERSION);}
}
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
  ecto::py::wrapDelay();
  ecto::py::wrap_ether();
  ecto::py:: wrap_dealer();

  bp::def("hardware_concurrency", &boost::thread::hardware_concurrency);

  // for setting breakpoints
  bp::def("dummy", &dummy);

  bp::def("version",&versionstr);
  bp::def("abi",&abinum);
  bp::def("soname",&sonametuple);

  // use this if you're embedding ipython and dont want to see
  // your cout/cerr
  bp::def("log_to_file", &ecto::py::log_to_file);
  bp::def("unlog_to_file", &ecto::py::unlog_to_file);
  ECTO_REGISTER(ecto);
}

