#include <ecto/ecto.hpp>
#include <iostream>
#include <string>
//start
using ecto::tendrils;
namespace introduction
{
  struct Printer01
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("prefix", "A string to prefix printing with.", "start>> ");
      params.declare<std::string>("suffix", "A string to append printing with.", " <<stop\n");
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<std::string>("message", "The message to print.");
    }
    void
    configure(tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      params["prefix"] >> prefix_;
      params["suffix"] >> suffix_;
    }
    int
    process(tendrils& inputs, tendrils& outputs)
    {
      std::cout << prefix_ << inputs.get<std::string>("message") << suffix_;
      return ecto::OK;
    }
    std::string prefix_, suffix_;
  };
}
//end

ECTO_CELL(introduction, introduction::Printer01, "Printer01",
          "A simple stdout printer with prefix and suffix parameters.");
