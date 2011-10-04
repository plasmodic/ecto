#pragma once
#include <ecto/ecto.hpp>
#include <ecto/cell.hpp>

#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/iterator.hpp>
#include <boost/python/slice.hpp>
#include <boost/python/stl_iterator.hpp>

#include <ecto/python/std_map_indexing_suite.hpp>
#include <ecto/python/raw_constructor.hpp>
#include <ecto/python/repr.hpp>
namespace ecto
{
  namespace py
  {
    namespace bp = boost::python;
    struct TendrilSpecification
    {
      cell::ptr mod_input, mod_output;
      std::string key;

      TendrilSpecification() { }

      bool
      check(cell::ptr mod, const std::string& key)
      {
        if (key.empty())
          return true;
        if (mod->inputs.find(key) == mod->inputs.end() && mod->outputs.find(key) == mod->outputs.end()
            && mod->parameters.find(key) == mod->parameters.end())
        {
          return false;
        }
        return true;
      }

      TendrilSpecification(cell::ptr mod_in, cell::ptr mod_out, const std::string& key)
          :
            mod_input(mod_in),
            mod_output(mod_out),
            key(key)
      {
        if (!check(mod_in, key))
          BOOST_THROW_EXCEPTION(except::EctoException() 
                                << except::diag_msg("no input or parameter found") 
                                << except::tendril_key(key) 
                                << except::cell_name(mod_in->name()));
        if (!check(mod_out, key))
          BOOST_THROW_EXCEPTION(except::EctoException() 
                                << except::diag_msg("no output or parameter found") 
                                << except::tendril_key(key) 
                                << except::cell_name(mod_in->name()));
      }

      TendrilSpecification(cell::ptr mod, const std::string& key)
          :
            mod_input(mod),
            mod_output(mod),
            key(key)
      {
        if (!check(mod, key))
          BOOST_THROW_EXCEPTION(except::EctoException() 
                                << except::diag_msg("no inputs or outputs found") 
                                << except::tendril_key(key) 
                                << except::cell_name(mod->name()));
      }

      tendril::ptr
      toTendril(tendril_type t)
      {
        switch (t)
        {
          case OUTPUT:
            return mod_output->outputs[key];
          case INPUT:
            return mod_input->inputs[key];
          case PARAMETER:
            return mod_input->parameters[key];
          default:
            return tendril::ptr();
        }
      }
      bp::str
      __str__()
      {
        bp::str str = bp::str(mod_input->name());
        str += ", " + bp::str(key);
        return str;
      }
    };

    struct TendrilSpecifications
    {
      typedef std::vector<TendrilSpecification> Vector;

      TendrilSpecifications()
      {
      }

      TendrilSpecifications(Vector vts)
          :
            vts(vts)
      {
      }

      TendrilSpecifications(bp::list l)
      {
        bp::stl_input_iterator<const TendrilSpecification&> begin(l), end;
        std::copy(begin, end, std::back_inserter(vts));
      }

      TendrilSpecification
      toSpec()
      {
        if (vts.size() != 1)
        {
          BOOST_THROW_EXCEPTION(except::EctoException() 
                                << except::diag_msg("This specification must be of length one. "
                                                    "e.g. module['only_one_key']"));
        }
        return vts.front();
      }

      static tendrils::ptr
      toTendrils(bp::dict d, int tt)
      {
        bp::list keys = d.keys();
        bp::stl_input_iterator<std::string> begin(keys), end;
        tendrils::ptr ts(new tendrils);

        while (begin != end)
        {
          std::string key = *begin;
          TendrilSpecifications spec = bp::extract<TendrilSpecifications>(d.get(bp::str(key)));
          tendril::ptr tp = spec.toSpec().toTendril(tendril_type(tt));
          ts->declare(key, tp);
          ++begin;

        }
        return ts;

      }
      Vector vts;
    };

    inline TendrilSpecifications
    getitem_str(cell::ptr mod, const std::string& key)
    {
      return TendrilSpecifications::Vector(1, TendrilSpecification(mod, key));
    }

    inline TendrilSpecifications
    getitem_tuple(cell::ptr mod, bp::tuple keys)
    {
      int end = bp::len(keys);
      TendrilSpecifications l;
      l.vts.reserve(end);
      for (int i = 0; i != end; ++i)
      {
        bp::extract<std::string> se(keys[i]);
        if (se.check())
          l.vts.push_back(TendrilSpecification(mod, se()));
        else
          throw std::runtime_error("All items must be str's");
      }
      return l;
    }

    inline TendrilSpecifications
    getitem_list(cell::ptr mod, bp::list keys)
    {
      bp::tuple t(keys);
      return getitem_tuple(mod, t);
    }

    inline TendrilSpecifications
    getitem_slice(cell::ptr mod, bp::slice s)
    {

      if (s == bp::slice())
      {
        return TendrilSpecifications::Vector(1, TendrilSpecification(mod, ""));
      }
      else
      {
        throw std::runtime_error("Slice is only valid if its the [:] form...");
      }
    }

    inline TendrilSpecifications
    expand(cell::ptr mod, const tendrils& t)
    {
      TendrilSpecifications l;

      BOOST_FOREACH(const tendrils::value_type& pair, t)
          {
            l.vts.push_back(TendrilSpecification(mod, pair.first));
          }
      return l;
    }

    inline bp::list
    rshift_spec(TendrilSpecifications& lhs, TendrilSpecifications& rhs)
    {
      bp::list result;
      if (lhs.vts.size() == 1 && lhs.vts.front().key.empty())
      {
        lhs = expand(lhs.vts.front().mod_output, lhs.vts.front().mod_output->outputs);
      }
      if (rhs.vts.size() == 1 && rhs.vts.front().key.empty())
      {
        rhs = expand(rhs.vts.front().mod_input, rhs.vts.front().mod_input->inputs);
      }
      //the spec must be the same size...
      if (lhs.vts.size() != rhs.vts.size())
      {
        std::string msg = boost::str(
            boost::format("Specification mismatch... len(lhs) != len(rhs) -> %d != %d") % lhs.vts.size()
            % rhs.vts.size());
        throw std::runtime_error(msg);
      }
      for (size_t i = 0, end = lhs.vts.size(); i < end; i++)
      {
        TendrilSpecification out = lhs.vts[i], in = rhs.vts[i];
        //check types, this will also assert on not found...
        out.mod_output->outputs[out.key]->compatible_type(*in.mod_input->inputs[in.key]);
        result.append(bp::make_tuple(out.mod_output, out.key, in.mod_input, in.key));
      }
      return result;
    }

    inline bp::list
    rshift_spec_tuples(TendrilSpecifications& lhs, bp::tuple& rhs)
    {
      bp::list result;
      bp::stl_input_iterator<TendrilSpecifications&> begin(rhs), end;
      while (begin != end)
      {
        result.extend(rshift_spec(lhs, *begin));
        ++begin;
      }
      return result;
    }
  }
}
