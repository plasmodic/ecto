#include <ecto/tendrils.hpp>
#include <boost/algorithm/string.hpp>

namespace ecto
{

  struct print_tendril
  {
    print_tendril(std::ostream& ss) :
      ss(ss)
    {
    }
    void operator()(const std::pair<std::string, ecto::tendril::ptr>& tp)
    {
      //default value
      std::string defval = "??? Todo FIXME";
      //            boost::python::extract<std::string>(
      //                                                boost::python::str(
      //                                                                   tp.second->extract()));
      ss << " - " << tp.first << " [" << tp.second->type_name()
          << "] default = " << defval << "\n";
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
