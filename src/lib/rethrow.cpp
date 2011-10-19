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
#include <ecto/rethrow.hpp>
#include <ecto/python.hpp>

namespace ecto {
  namespace except {
    namespace py {

      boost::exception_ptr rethrowable_in_interpreter_thread;

      int rethrow_in_python(void * val)
      {
        ECTO_LOG_DEBUG("IT IS CALLED!!! %p", val);
        boost::python::handle_exception(boost::bind(&boost::rethrow_exception, rethrowable_in_interpreter_thread));
        //  boost::rethrow_exception(eptr);
        return -1;
      }

      void rethrow_schedule()
      {
        PyGILState_STATE gstate;
        gstate = PyGILState_Ensure();

        rethrowable_in_interpreter_thread = boost::current_exception();
        Py_AddPendingCall(&rethrow_in_python, (void*)13);
        /* Release the thread. No Python API allowed beyond this point. */
        PyGILState_Release(gstate);
      }

    }
  }
}

