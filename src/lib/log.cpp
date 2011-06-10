#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost;

namespace ecto {
  mutex log_mtx;

  void log(const std::string& msg)
  {
    posix_time::ptime now(posix_time::microsec_clock::local_time());
    
    std::cout << now << " " << msg << std::endl;
  }
}
