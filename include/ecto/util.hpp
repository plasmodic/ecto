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

namespace ecto {
  std::string name_of(const std::type_info &ti);

  template <typename T>
  std::string name_of()
  {
    return name_of(typeid(T));
  }

}
#endif
