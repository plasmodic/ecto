#include <ecto/module.hpp>
#include <ecto/util.hpp>
#include <ecto/except.hpp>
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
    dispatch_configure(parameters, inputs, outputs);
  }

  ReturnCode module::process()
  {
    //trigger all parameter change callbacks...
    tendrils::iterator begin = parameters.begin(), end = parameters.end();
    while (begin != end)
    {
      begin->second->notify();
      ++begin;
    }

    try
    {
      return dispatch_process(inputs, outputs);
    } catch (std::exception& e)
    {
      throw std::runtime_error("Module " + name() + " threw\n" + e.what());
    }
  }

  std::string module::type() const
  {
    return dispatch_name();
  }

  void module::name(const std::string& name)
  {
    instance_name = name;
  }

  std::string module::name() const
  {
    return instance_name.size() ? instance_name : dispatch_name();
  }

  void module::destroy()
  {
    dispatch_destroy();
  }

  std::string module::gen_doc(const std::string& doc) const
  {
    std::stringstream ss;

    ss << name() << " (ecto::module)\n";
    //create an underline that is the size of the name...
    for (int i = 0, end = ss.str().size(); i < end; ++i)
    {
      ss << "=";
    }
    ss << "\n";
    ss << "\n" << doc << "\n\n";
    parameters.print_doc(ss, "Parameters");
    inputs.print_doc(ss, "Inputs");
    outputs.print_doc(ss, "Outputs");
    return ss.str();
  }

  void module::verify_params() const
  {

    tendrils::const_iterator it = parameters.begin(), end(parameters.end());
    while (it != end)
    {
      if (it->second->is_required() && !it->second->user_supplied())
      {
        throw except::ValueRequired(it->first + " must be supplied with a value during initialization.");
      }
      ++it;
    }
  }

}
