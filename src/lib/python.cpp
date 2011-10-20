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
#include <ecto/python.hpp>
#include <ecto/python/repr.hpp>
#include <ecto/python/gil.hpp>
#include <ecto/log.hpp>

namespace ecto {
  namespace py {
    std::string repr(const boost::python::object& obj)
    {
      return boost::python::extract<std::string>(obj.attr("__repr__")());
    }

    struct gil::impl : boost::noncopyable
    {
      PyGILState_STATE gstate;
    };

    gil::gil() : impl_(new gil::impl)
    {
      //impl_->gstate = PyGILState_Ensure();
    }

    gil::~gil()
    {
      //PyGILState_Release(impl_->gstate);
    }


    PyThreadState* scoped_gil_release::threadstate = NULL;

    scoped_gil_release::scoped_gil_release()
    {
      if (!Py_IsInitialized())
        return;
      if (threadstate)
        mine = false;
      else
        {
          threadstate = PyEval_SaveThread();
          mine = true;
        }
    }

    scoped_gil_release::~scoped_gil_release() {
      if (!Py_IsInitialized())
        return;
      if (mine) {
        PyEval_RestoreThread(threadstate);
        mine = false;
        threadstate = NULL;
      }
    }

    scoped_call_back_to_python::scoped_call_back_to_python()
      : have(false)
    {
      if (!Py_IsInitialized())
        return;

      have = true;
      gilstate = PyGILState_Ensure();
    }

    scoped_call_back_to_python::~scoped_call_back_to_python()
    {
      if (!Py_IsInitialized())
        return;
      ECTO_ASSERT(have, "We have no GIL to release");
      PyGILState_Release(gilstate);
    }

  }
}

