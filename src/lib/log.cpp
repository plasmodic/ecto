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
#include <ecto/except.hpp>
#include <ecto/test.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

using namespace boost;

namespace ecto {

  bool logging_on() {
    static bool val = getenv("ECTO_LOGGING");
    return val;
  }

  mutex log_mtx;
  mutex process_log_mtx;

  const static std::string srcdir(SOURCE_DIR);
  const static unsigned srcdirlen(srcdir.size()+1);

  void log(const char* file, unsigned line, const std::string& msg)
  {
    mutex::scoped_lock lock(log_mtx);
    posix_time::ptime now(posix_time::microsec_clock::local_time());
    const char* file_remainder = file + srcdirlen;
    std::cout << str(boost::format("%14p %40s:%-4u ") % boost::this_thread::get_id() % file_remainder % line)
              << msg << std::endl;

  }

  void assert_failed(const char* file, unsigned line, const char* cond, const char* msg)
  {
    log(file, line, str(boost::format("ASSERT FAILED: %s (%s)") % cond % msg));
    abort();
  }

}

