#pragma once
#include <ecto/tendril.hpp>

#include <boost/python.hpp>


namespace bp = boost::python;

namespace
{
using ecto::tendril;

struct Setter
{
  Setter(tendril::ptr ot, bp::object obj)
  {
    proxy.copy_value(*ot);//grabs type info.
    proxy.set(obj);//copy value from the bp object.
  }
  void operator()(tendril& t)
  {
    t.copy_value(proxy);
  }
  tendril proxy;
};
}
