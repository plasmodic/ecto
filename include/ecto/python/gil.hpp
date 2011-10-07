#pragma once

#include <boost/scoped_ptr.hpp>

namespace ecto {
  namespace py {

    class gil : boost::noncopyable
    {
      struct impl;
      boost::scoped_ptr<impl> impl_;

    public:
      gil();
      ~gil();
    };

    class nothing_to_lock : boost::noncopyable
    {
    public:
      nothing_to_lock() { }
      ~nothing_to_lock() { }
    };

  }
}
