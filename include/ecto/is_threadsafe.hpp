#pragma once

#include <boost/mpl/bool.hpp>

namespace ecto {
  namespace detail {
    template <typename T> struct is_thread_unsafe : boost::mpl::false_ { };
  }
}


#define ECTO_THREAD_UNSAFE(T)                                           \
  namespace ecto {                                                      \
    namespace detail {                                                  \
      template <> struct is_thread_unsafe<T> : boost::mpl::true_ { };   \
    }                                                                   \
  }                                                                     \

