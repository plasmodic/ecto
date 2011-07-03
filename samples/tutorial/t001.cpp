#include <ecto/ecto.hpp>
#include <iostream>
#include <string>

//name, docstr, default_value = T()
//Note: all tendril objects have a valid default value
//We'll use the tendrils in here, as they hold the current value in 
//in.read<T>("key") is an explicit const read of the value held at "key", a reference to const T will be returned
//in.get<std::string>("input") is also acceptible but may be more ambiguous.
//read and get will not cause a copy to occur.
namespace tutorial
{
  using ecto::tendrils;

  struct Printer02
  {
    static void 
    declare_io(const tendrils& /*params*/, tendrils& in, tendrils& /*out*/)
    {
      in.declare<std::string>("input", "A string to print", "Default Value");
    }
    
    int
    process(const tendrils& in, tendrils& /*out*/)
    {
      std::cout << in.read<std::string>("input") << std::endl;
      return ecto::OK;
    }
  };
  
  struct Reader01
  {
    static void 
    declare_io(const tendrils& /*params*/, tendrils& /*in*/, tendrils& out)
    {
      out.declare<std::string>("output", "A string read from standard input."); //NOTE: no default value
    }
    
    int
    process(const tendrils& /*in*/, tendrils& out)
    {
      std::cin >> out.get<std::string>("output");
      return ecto::OK;
    }
  };

}

//register our cell with the existing ecto cell 'tutorial' that is declared in tutorial.cpp
ECTO_CELL(tutorial, tutorial::Printer02, "Printer02", "Prints a string to standard output.");
ECTO_CELL(tutorial, tutorial::Reader01, "Reader01",  "Reads a string from standard input.");
