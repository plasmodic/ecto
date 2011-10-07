#pragma once

#include <boost/mpl/bool.hpp>
#include <ecto/python/gil.hpp>

namespace ecto {
  namespace detail {
    template <typename T> struct is_threadsafe : boost::mpl::true_ { };

    template <typename T> struct python_mutex 
    { 
      typedef ecto::py::nothing_to_lock type; 
    };
  }
}


#define ECTO_THREAD_UNSAFE(T)                                           \
  namespace ecto {                                                      \
    namespace detail {                                                  \
      template <> struct is_threadsafe<T> : boost::mpl::false_ { };   \
    }                                                                   \
  }                                                                     \


#define ECTO_NEEDS_PYTHON_GIL(T)                                        \
  namespace ecto {                                                      \
    namespace detail {                                                  \
      template <> struct python_mutex<T> {                              \
        typedef ecto::py::gil type;                                     \
      };                                                                \
    }                                                                   \
  }                                                                     \



