#pragma once
#include <boost/format.hpp>

namespace ecto {
  void log(const std::string& msg);
  extern boost::mutex log_mtx;
}

#if 0
#define ECTO_LOG_DEBUG(fmt, args)                                       \
  do {                                                                  \
    log_mtx.lock();                                                     \
    log(str(boost::format(fmt) % args));                                \
    log_mtx.unlock();                                                   \
  } while (false)
#else
#define ECTO_LOG_DEBUG(fmg, args) do { } while (false)
#endif
