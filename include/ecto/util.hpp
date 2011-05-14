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

namespace ecto
{
/**
 * \brief Get the unmangled type name of a type_info object.
 * @param ti The type_info to look up unmangled name for.
 * @return The unmangled name. e.g. cv::Mat or pcl::PointCloud<pcl::PointXYZ>
 */
std::string name_of(const std::type_info &ti);

/**
 * \brief Get the unmangled type name of a type.
 * @tparam T the type that one wants a name for.
 * @return The unmangled name of the given type.
 */
template<typename T>
std::string name_of()
{
  return name_of(typeid(T));
}

}
