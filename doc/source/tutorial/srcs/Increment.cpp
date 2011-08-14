#include <ecto/ecto.hpp>
#include <iostream>
#include <string>

namespace tutorial
{
  using ecto::tendrils;

  struct Increment
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<int>("start", "The starting value.", 0);
    }

    static void
    declare_io(const tendrils& params, tendrils& /*in*/, tendrils& out)
    {
      int start;
      params["start"] >> start;//copies the value into a local variable
      out.declare<int>("output", "An increasing integer", start);
    }

    int
    process(const tendrils& in, const tendrils& out)
    {
      //the get function of the tendrils returns a mutable reference.
      out.get<int>("output") += 1;
      return ecto::OK;
    }
  };
}

ECTO_CELL(tutorial, tutorial::Increment, "Increment", "Outputs a monotonically increasing integer.");
