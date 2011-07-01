#include <ecto/ecto.hpp>
#include <iostream>

namespace tutorial
{
  using ecto::tendrils;

  struct Printer01
  {
    int
    process(const tendrils& /*in*/, tendrils& /*out*/)
    {
      std::cout << "Hello" << std::endl;
      return ecto::OK;
    }
  };

}

//register our cell with the existing ecto module 'tutorial' that is declared in tutorial.cpp
ECTO_MODULE(tutorial, tutorial::Printer01, "Printer01", "Prints 'Hello' to standard output.");
