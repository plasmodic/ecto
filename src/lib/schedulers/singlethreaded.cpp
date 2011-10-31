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

#include <ecto/util.hpp>
#include <ecto/plasm.hpp>
#include <ecto/schedulers/singlethreaded.hpp>

namespace ecto {

  namespace schedulers {

    singlethreaded::singlethreaded(plasm_ptr p)
      : scheduler(p),interupted_(false)
    { }

    singlethreaded::~singlethreaded()
    {
      if (!running())
        return;
      interrupt();
      wait();
    }

    int singlethreaded::execute_impl(unsigned niter, unsigned nthread, boost::asio::io_service& topserv)
    {
      ECTO_START();
      interupted_ = false;
      profile::graphstats_collector gs(graphstats);

      size_t retval = ecto::OK;
      unsigned cur_iter = 0;
      while((niter == 0 || cur_iter < niter))
        {
          for (size_t k = 0; k < stack.size(); ++k)
            {
              if(interupted_){
                return ecto::QUIT;//someone interrupted.
              }
              ECTO_LOG_DEBUG("k=%u niter=%u", k % niter);
              //need to check the return val of a process here, non zero means exit...
              retval = invoke_process(stack[k]);
              if (retval) {
                return retval;
              }
            }
          ++cur_iter;
        }
      ECTO_LOG_DEBUG("FINISH %s", __PRETTY_FUNCTION__);
      return retval;
    }

    void singlethreaded::interrupt_impl()
    {
      interupted_ = true;
    }

    void singlethreaded::stop_impl()
    {
      //nothing special to do here.. invoke_process takes care of this.
    }
    void singlethreaded::wait_impl()
    {
    }
  }
}

