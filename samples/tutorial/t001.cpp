#include <ecto/ecto.hpp>
#include <iostream>
#include <string>

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
      //NOTE: no default value
      out.declare<std::string>("output", "A string read from standard input.");
    }
    
    int
    process(const tendrils& /*in*/, tendrils& out)
    {
      std::cin >> out.get<std::string>("output");
      //add a quit condition
      if(out.get<std::string>("output") == "q")
        return ecto::QUIT;
      return ecto::OK;
    }
  };

}

//register our cell with the existing ecto cell 'tutorial' that is declared in tutorial.cpp
ECTO_CELL(tutorial, tutorial::Printer02, "Printer02", "Prints a string to standard output.");
ECTO_CELL(tutorial, tutorial::Reader01, "Reader01",  "Reads a string from standard input.");