#ifndef ECTO_UTIL_HPP_INCLUDED
#define ECTO_UTIL_HPP_INCLUDED
#include <typeinfo>
#include <string>
#include <vector>
#include <iostream>
#include <stdint.h>

#include <typeinfo>
#include <string>

#define SHOW() (std::cout << __PRETTY_FUNCTION__ << "\n")

//not sure if we should disable this even in release...
//#if NDEBUG
#if 0
#define ECTO_ASSERT(_impl_check_ )        \
  do {} while(false)
#else
#define ECTO_ASSERT(_impl_check_ )\
  do\
    {\
      _impl_check_;\
    }while(false)
#endif

namespace ecto {
  std::string name_of(const std::type_info &ti);

  template <typename T>
  std::string name_of()
  {
    return name_of(typeid(T));
  }

}
#endif
