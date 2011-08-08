#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

using namespace boost;

namespace ecto {
  mutex log_mtx;
  mutex process_log_mtx;

  void log(const std::string& msg)
  {
    mutex::scoped_lock lock(log_mtx);
    posix_time::ptime now(posix_time::microsec_clock::local_time());
    std::cout << now << " " << boost::this_thread::get_id() << " " << msg << std::endl;
  }

#if defined(ECTO_LOG_STATS)
  std::ofstream processlog("process.log");

  void log_process(const std::string& instancename, uint64_t time, unsigned ncalls, bool onoff)
  {
    mutex::scoped_lock lock(process_log_mtx);
    processlog << "process " << instancename << " " << time << " " << ncalls << " " << onoff << "\n";
  }
#endif

}
