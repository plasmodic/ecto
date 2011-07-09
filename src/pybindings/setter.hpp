#pragma once
#include <ecto/tendril.hpp>

#include <boost/python.hpp>


namespace bp = boost::python;

namespace
{
using ecto::tendril;

struct Setter
{
  Setter(tendril::ptr ot, bp::object obj):lt(ot)
  {
    t.copy_value(*ot);//grabs type info.
    t.set(obj);//copy value from the bp object.
  }
  void operator()()
  {
    lt->copy_value(t);
  }
  tendril::ptr lt;
  tendril t;
};
}
