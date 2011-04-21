#pragma once
#include <typeinfo>
#include <string>
#include <vector>
#include <iostream>
#include <stdint.h>

#include <typeinfo>
#include <string>

#if !defined(DISABLE_SHOW)
#define SHOW() std::cout << __PRETTY_FUNCTION__ << "\n"
#else
#define SHOW() do{}while(false)
#endif

namespace ecto {
  std::string name_of(const std::type_info &ti);

  template <typename T>
  std::string name_of()
  {
    return name_of(typeid(T));
  }

}
