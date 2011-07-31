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
    proxy = *ot;
    proxy << obj;//copy value from the bp object.
  }
  void operator()(tendril& t)
  {
    t << proxy;
    t.dirty(true);
    t.user_supplied(true);
  }
  tendril proxy;
};
}
