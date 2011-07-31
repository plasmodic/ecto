#include <ecto/cell.hpp>
#include <ecto/util.hpp>
#include <ecto/except.hpp>
#include <boost/exception/all.hpp>

/*
 * Catch all and pass on exception.
 */
#define CATCH_ALL() \
catch (ecto::except::NonExistant& e) \
{ \
  auto_suggest(e,*this); \
  e << "  Module : " + name() + "\n  Function: " + __FUNCTION__; \
  boost::throw_exception(e); \
} \
catch (ecto::except::EctoException& e) \
{ \
  e << "  Module : " + name() + "\n  Function: " + __FUNCTION__; \
  boost::throw_exception(e); \
} catch (std::exception& e) \
{ \
  except::EctoException ee("Original Exception: " +name_of(typeid(e))); \
  ee << "  What   : " + std::string(e.what()); \
  ee << "  Module : " + name() + "\n  Function: " + __FUNCTION__; \
  boost::throw_exception(ee); \
} \
catch (...) \
{ \
  except::EctoException ee("Threw unknown exception type!"); \
  ee << "  Module : " + name() + "\n  Function: " + __FUNCTION__; \
  boost::throw_exception(ee); \
}

namespace ecto
{
  void
  auto_suggest(except::NonExistant& e, const cell& m)
  {
    std::string p_type, i_type, o_type;
    bool in_p = m.parameters.find(e.key) != m.parameters.end();
    if(in_p) p_type = m.parameters.find(e.key)->second->type_name();

    bool in_i = m.inputs.find(e.key) != m.inputs.end();
    if(in_i) i_type = m.inputs.find(e.key)->second->type_name();

    bool in_o = m.outputs.find(e.key) != m.outputs.end();
    if(in_o) o_type = m.outputs.find(e.key)->second->type_name();

    if (in_p || in_i || in_o)
    {
      e << ("  Hint   : '" + e.key + "' does exist in " + (in_p ? "parameters (type == " +p_type +") " : "") + (in_i ? "inputs (type == " +i_type +") "  : "")
          + (in_o ? "outputs (type == " +o_type +")" : ""));
    }
    else
    {
      e << ("  Hint   : '" + e.key + "' does not exist in module.");
    }
  }
  cell::cell()
  {
  }

  cell::~cell()
  {
  }

  void
  cell::declare_params()
  {
    try
    {
      dispatch_declare_params(parameters);
    }CATCH_ALL()
  }

  void
  cell::declare_io()
  {
    try
    {
      dispatch_declare_io(parameters, inputs, outputs);
    }CATCH_ALL()
  }

  void
  cell::configure()
  {
    try
    {
      dispatch_configure(parameters, inputs, outputs);
    }CATCH_ALL()
  }

  ReturnCode
  cell::process()
  {
    init();
    //trigger all parameter change callbacks...
    tendrils::iterator begin = parameters.begin(), end = parameters.end();

    while (begin != end)
    {
      try
      {
        begin->second->notify();
      } catch (const std::exception& e)
      {
        except::EctoException ee("Original Exception: " + name_of(typeid(e)));
        ee << "  What   : " + std::string(e.what());
        ee << "  Module : " + name() + "\n  Function: Parameter Callback for '" + begin->first + "'";
        boost::throw_exception(ee);
      }
      ++begin;
    }

    try
    {
      return dispatch_process(inputs, outputs);
    }CATCH_ALL()
  }

  void
  cell::destroy()
  {
    try
    {
      dispatch_destroy();
    }CATCH_ALL()
  }

  std::string
  cell::type() const
  {
    return dispatch_name();
  }

  void
  cell::name(const std::string& name)
  {
    instance_name_ = name;
  }

  std::string
  cell::name() const
  {
    return instance_name_.size() ? instance_name_ : dispatch_name();
  }

  void
  cell::short_doc(const std::string& short_doc)
  {
    dispatch_short_doc(short_doc);
  }

  std::string
  cell::short_doc() const
  {
    return dispatch_short_doc();
  }

  std::string
  cell::gen_doc(const std::string& doc) const
  {
    std::stringstream ss;
    ss << name() << " (ecto::module):\n";
    ss << "\n";
    ss << "\n" << doc << "\n\n";
    parameters.print_doc(ss, "Parameters");
    inputs.print_doc(ss, "Inputs");
    outputs.print_doc(ss, "Outputs");
    return ss.str();
  }

  void
  cell::verify_params() const
  {
    tendrils::const_iterator it = parameters.begin(), end(parameters.end());
    while (it != end)
    {
      if (it->second->required() && !it->second->user_supplied())
      {
        throw except::ValueRequired("parameter '" + it->first + "' must be supplied with a value during initialization.");
      }
      ++it;
    }
  }
  
  void
  cell::verify_inputs() const
  {
    tendrils::const_iterator it = inputs.begin(), end(inputs.end());
    while (it != end)
    {
      if (it->second->required() && !it->second->user_supplied())
      {
        throw except::ValueRequired("'input' " + it->first + "' must be connected.");
      }
      ++it;
    }
  }

}
