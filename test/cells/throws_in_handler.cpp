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

#define ECTO_LOG_ON
#include <ecto/log.hpp>
#include <ecto/except.hpp>
#include <ecto/python.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

using namespace ecto::except;


void boom(const boost::system::error_code&)
{
  ECTO_START();
  BOOST_THROW_EXCEPTION(EctoException()
                        << diag_msg("boom: thrown from an io_service in a thread in the bg"));
  ECTO_FINISH();
}

boost::exception_ptr eptr;

int something_is_up(void * val)
{
  ECTO_LOG_DEBUG("IT IS CALLED!!! %p", val);
  boost::python::handle_exception(boost::bind(&boost::rethrow_exception, eptr));
  //  boost::rethrow_exception(eptr);
  return -1;
}

struct throws_in_bg
{
  boost::asio::io_service serv;
  boost::asio::io_service::work work;
  boost::asio::deadline_timer dt;
  boost::thread runthread;

  throws_in_bg()
    : work(serv),
      dt(serv, boost::posix_time::seconds(1))
  {
    ECTO_START();
    PyEval_InitThreads();
    ECTO_ASSERT(PyEval_ThreadsInitialized(), "threads not initialized, uh oh");
    // Py_AddPendingCall(&something_is_up, (void*)13);
    dt.async_wait(&boom);

    boost::thread tmp(boost::bind(&throws_in_bg::bgthread, this));
    tmp.swap(runthread);
    ECTO_FINISH();
  }

  void bgthread()
  {
    ECTO_START();
    try {
      serv.run();
    } catch (const boost::exception& e) {
      // serv.stop();
      ECTO_LOG_DEBUG("%s", "Caught...");
      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();

      eptr = boost::current_exception();
      Py_AddPendingCall(&something_is_up, (void*)13);
      /* Release the thread. No Python API allowed beyond this point. */
      PyGILState_Release(gstate);
      //throw;
    }
    ECTO_FINISH();
  }


};


namespace {
  boost::shared_ptr<throws_in_bg> throwptr;
}

void should_rethrow_in_interpreter_thread()
{
  throwptr.reset(new throws_in_bg);
  std::cout << "throwptr = " << throwptr.get() << "\n";
}

void should_throw_in_interpreter_thread()
{
  boom(boost::system::error_code());
}
