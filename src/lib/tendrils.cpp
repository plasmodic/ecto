#include <ecto/tendrils.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/function.hpp>
#include <map>
#include <iostream>
namespace ecto
{
  struct PrintFunctions
  {
    template <typename T>
    static void print(std::ostream& out,const tendril& x)
    {
      out << x.read<T>();
    }

    typedef std::map<std::string, boost::function<void(std::ostream& out,const tendril& x)> > ProcMap;
    ProcMap processes;
    PrintFunctions()
    {
      processes[ecto::name_of<int>()] = &print<int>;
      processes[ecto::name_of<float>()] = &print<float>;
      processes[ecto::name_of<double>()] = &print<double>;
      processes[ecto::name_of<bool>()] = &print<bool>;
      processes[ecto::name_of<std::string>()] = &print<std::string>;
    }

    void print_tendril(std::ostream& out, const tendril& t) const
    {
      ProcMap::const_iterator it = processes.find(t.type_name());
      if(it != processes.end())
      {
        it->second(out,t);
      }
      else
      {
        out << t.type_name() << "(?)";
      }
    }
  };

  const PrintFunctions pf;

  struct print_tendril
  {
    print_tendril(std::ostream& ss) :
      ss(ss)
    {
    }
    void operator()(const std::pair<std::string, ecto::tendril::ptr>& tp)
    {
      std::stringstream tss;
      pf.print_tendril(tss,*tp.second);
      //default value

      ss << " - " << tp.first << " [" << tp.second->type_name() << "]";
      ss << (tp.second->has_default() ? (" default = " + tss.str()) : "");
      ss << (tp.second->is_required() ? " REQUIRED " : "");
      ss << "\n";

      std::string docstr = tp.second->doc();
      std::vector<std::string> doc_lines;
      std::string doc_str = tp.second->doc();
      boost::split(doc_lines, doc_str, boost::is_any_of("\n"));
      for (size_t i = 0; i < doc_lines.size(); ++i)
        ss << "    " << doc_lines[i] << "\n";
      ss << "\n";
    }
    std::ostream& ss;
  };

  void tendrils::print_doc(std::ostream& out, const std::string& tendrils_name) const
  {
    boost::mutex::scoped_lock lock(mtx);
    if (empty())
      return;
    out << tendrils_name << "\n";
    out << "---------------------------------\n\n";
    std::for_each(begin(), end(), print_tendril(out));
  }

  tendril::const_ptr tendrils::at(const std::string& name) const
  {
    boost::mutex::scoped_lock lock(mtx);
    map_t::const_iterator it = find(name);
    if (it == end())
      throw std::logic_error(name + " does not exist!");
    return it->second;
  }

  tendril::ptr tendrils::at(const std::string& name)
  {
    boost::mutex::scoped_lock lock(mtx);
    map_t::iterator it = find(name);
    if (it == end())
      throw std::logic_error(name + " does not exist!");
    return it->second;
  }
}
