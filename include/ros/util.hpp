#ifndef ROS_UTIL_HPP_INCLUDED
#define ROS_UTIL_HPP_INCLUDED

#include <typeinfo>
#include <string>
#include <iosfwd>

std::string name_of(const std::type_info &ti);

template <typename T>
std::string name_of()
{
  return name_of(typeid(T));
}

#define SHOW(...) std::cout << __PRETTY_FUNCTION__ << " " << __VA_ARGS__ << "\n"

#endif
