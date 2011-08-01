#pragma once

#include <Python.h>
#include <boost/thread/locks.hpp>

namespace ecto {

  class gil : boost::noncopyable
  {
  private:
    PyGILState_STATE gstate;

  public:
    gil() { gstate = PyGILState_Ensure(); }
    
    ~gil() { PyGILState_Release(gstate);   }
  };

  class nothing_to_lock : boost::noncopyable
  {
  public:
    nothing_to_lock() { }
    ~nothing_to_lock() { }
  };

}
