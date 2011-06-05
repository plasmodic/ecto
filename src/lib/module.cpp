#include <ecto/module.hpp>
#include <ecto/util.hpp>

namespace ecto
{
  module::module()
  {
  }

  module::~module()
  {
  }

  void module::declare_params()
  {
    dispatch_declare_params(parameters);
  }

  void module::declare_io()
  {
    dispatch_declare_io(parameters, inputs, outputs);
  }

  void module::configure()
  {
    dispatch_configure(parameters);
  }

  ReturnCode module::process()
  {
    try
      {
        //trigger all parameter change callbacks...
        tendrils::iterator begin = parameters.begin(), end = parameters.end();
        while (begin != end)
          {
            begin->second.trigger_callback();
            ++begin;
          }
        return dispatch_process(inputs, outputs);
      } catch (std::exception& e)
      {
        throw std::runtime_error(name() + " threw an exception :\n" + e.what());
      }
  }

  std::string module::name() const
  {
    return dispatch_name();
  }

  void module::destroy()
  {
    dispatch_destroy();
  }

  std::string module::gen_doc(const std::string& doc) const
  {
    std::stringstream ss;
    ss << name() << " (ecto::module)\n";
    ss << "===============================\n";
    ss << "\n" << doc << "\n\n";
    parameters.print_doc(ss, "Parameters");
    inputs.print_doc(ss, "Inputs");
    outputs.print_doc(ss, "Outputs");
    return ss.str();
  }

}
