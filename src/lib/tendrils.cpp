#include <ecto/tendrils.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/function.hpp>
#include <map>
#include <iostream>
namespace ecto
{
  struct PrintFunctions
  {
    template<typename T>
    static void
    print(std::ostream& out, const tendril& x)
    {
      out << x.get<T>();
    }

	typedef boost::function<void(std::ostream& out, const tendril& x)> function_t;
    typedef std::map<std::string,function_t> ProcMap;
    ProcMap processes;
    PrintFunctions()
    {
      processes[ecto::name_of<int>()] = function_t(&PrintFunctions::print<int>);
      processes[ecto::name_of<float>()] = function_t(&PrintFunctions::print<float>);
      processes[ecto::name_of<double>()] = function_t(&PrintFunctions::print<double>);
      processes[ecto::name_of<bool>()] = function_t(&PrintFunctions::print<bool>);
      processes[ecto::name_of<std::string>()] = function_t(&PrintFunctions::print<std::string>);
    }

    void
    print_tendril(std::ostream& out, const tendril& t) const
    {
      ProcMap::const_iterator it = processes.find(t.type_name());
      if (it != processes.end())
      {
        it->second(out, t);
      }
      else
      {
        out << t.type_name() << "(?)";
      }
    }
  };

  const PrintFunctions pf;
  struct print_tendril_simple
  {
    print_tendril_simple(std::ostream& ss)
      : ss(ss)
    { }
    void
    operator()(const std::pair<std::string, ecto::tendril::ptr>& tp)
    {
      ss << " '" << tp.first << "':type(" << tp.second->type_name() << ")";
    }
    std::ostream& ss;
  };

  struct print_tendril
  {
    print_tendril(std::ostream& ss)
      : ss(ss)
    { }

    void
    operator()(const std::pair<std::string, ecto::tendril::ptr>& tp)
    {
      std::stringstream tss;
      pf.print_tendril(tss, *tp.second);
      //default value

      ss << " - " << tp.first << " [" << tp.second->type_name() << "]";
      ss << (tp.second->has_default() ? (" default = " + tss.str()) : "");
      ss << (tp.second->required() ? " REQUIRED " : "");
      ss << "\n";

      std::string docstr = tp.second->doc();
      std::vector<std::string> doc_lines;
      std::string doc_str = tp.second->doc();
      boost::split(doc_lines, doc_str, boost::is_any_of(std::string("\n")));//get rid of warning on earlier versions of boost std::string("\n")
      for (size_t i = 0; i < doc_lines.size(); ++i)
        ss << "    " << doc_lines[i] << "\n";
      ss << "\n";
    }
    std::ostream& ss;
  };

  //////////////////////////////////////////////////////////////////////////////

  tendrils::tendrils() { }

  void
  tendrils::print_doc(std::ostream& out, const std::string& tendrils_name) const
  {
    boost::mutex::scoped_lock lock(mtx);
    if (storage.empty())
      return;
    out << tendrils_name << ":\n";
    // out << "---------------------------------\n\n";
    std::for_each(storage.begin(), storage.end(), print_tendril(out));
  }

  void
  tendrils::doesnt_exist(const std::string& name) const
  {
    std::stringstream ss;
    std::for_each(begin(),end(),print_tendril_simple(ss));
    BOOST_THROW_EXCEPTION(except::NonExistant()
                          << except::tendril_key(name)
                          << except::actualkeys_hint(ss.str()));
  }

  const tendril::ptr&
  tendrils::operator[](const std::string& name) const
  {
    boost::mutex::scoped_lock lock(mtx);
    storage_type::const_iterator it = storage.find(name);
    if (it == end())
      doesnt_exist(name);
    return it->second;
  }

  tendril::ptr&
  tendrils::operator[](const std::string& name)
  {
    boost::mutex::scoped_lock lock(mtx);
    storage_type::iterator it = storage.find(name);
    if (it == end())
      doesnt_exist(name);
    return it->second;
  }

  tendril::ptr
  tendrils::declare(const std::string& name, tendril::ptr t)
  {
    storage_type::iterator it = find(name);
    //if there are no exiting tendrils by the given name,
    //just add it.
    if (it == end())
    {
      storage.insert(std::make_pair(name, t));
    }
    else // we want to just return the existing tendril (so that
         // modules preconnected don't get messed up)...
    {
        BOOST_THROW_EXCEPTION(except::TendrilRedeclaration()
                              << except::tendril_key(name)
                              << except::prev_typename(it->second->type_name())
                              << except::cur_typename(t->type_name()));
          ;
    }
    return storage.at(name);
  }
}
