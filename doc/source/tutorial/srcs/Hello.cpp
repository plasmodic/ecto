#include <ecto/ecto.hpp>
#include <iostream>

namespace tutorial
{
  using ecto::tendrils;
  struct Hello
  {
    int
    process(const tendrils& /*in*/, const tendrils& /*out*/)
    {
      std::cout << "Hello" << std::endl;
      return ecto::OK;
    }
  };
}

ECTO_CELL(tutorial, tutorial::Hello, "Hello", "Prints 'Hello' to standard output.");
