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

#include <ecto/ecto.hpp>

#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/cell.hpp>
#include <ecto/log.hpp>
#include <ecto/strand.hpp>

#include <ecto/impl/graph_types.hpp>
#include <ecto/edge.hpp>
#include <ecto/impl/invoke.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>

#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>
#include <boost/format.hpp>

#include <boost/spirit/home/phoenix/core.hpp>
#include <boost/spirit/home/phoenix/operator.hpp>
#include <boost/exception/all.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace ecto {

  namespace schedulers {
    namespace asio = boost::asio;

    namespace pt = boost::posix_time;
    using boost::exception_ptr;
    using boost::shared_ptr;
    using boost::thread;

    struct propagator
    {
      asio::io_service &from, &to;
      asio::io_service::work work;

      propagator(asio::io_service& from_, asio::io_service& to_)
        : from(from_), to(to_), work(to) { }

      void operator()() {
        from.run();
      }

      template <typename Handler>
      void post(Handler h)
      {
        to.post(h);
      }
    };

    struct thrower
    {
      exception_ptr eptr;
      thrower(exception_ptr eptr_) : eptr(eptr_) { }

      void operator()() const
      {
        ECTO_LOG_DEBUG("%s", __PRETTY_FUNCTION__);
        rethrow_exception(eptr);
      }
    };

    struct runandjoin
    {
      typedef shared_ptr<runandjoin> ptr;

      thread runner;

      runandjoin() { }

      ~runandjoin() {
        if (runner.joinable())
          runner.join();
      }

      void join()
      {
        if (runner.joinable())
          runner.join();
      }

      void interrupt()
      {
        ECTO_LOG_DEBUG("interrupting %p", this);
        runner.interrupt();
        ECTO_LOG_DEBUG("interrupt    %p done", this);
      }

      template <typename Work>
      void impl(Work w)
      {
        try {
          w();
        } catch (const boost::exception& e) {
          ECTO_LOG_DEBUG("post thrower(boost::exception) %s", __PRETTY_FUNCTION__);
          w.post(thrower(boost::current_exception()));
        } catch (const std::exception& e) {
          ECTO_LOG_DEBUG("post thrower (std::exception) %s", __PRETTY_FUNCTION__);
          w.post(thrower(boost::current_exception()));
        }
        w.post(boost::bind(&runandjoin::join, this));
      }

      template <typename Work>
      void run(Work w)
      {
        boost::scoped_ptr<thread> newthread(new thread(boost::bind(&runandjoin::impl<Work>, this, w)));
        newthread->swap(runner);
      }
    };
  }
}


