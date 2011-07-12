#include <vector>
#include <iostream>
#include <ecto/util.hpp>

#include <cxxabi.h>
#include <string>
#include <stdlib.h>
#include <map>
#include <boost/thread/mutex.hpp>
#include <boost/tuple/tuple.hpp>
namespace ecto
{

  struct type_mapping
  {
    type_mapping()
    {
    }
    const std::string&
    lookup(const std::type_info& ti)
    {
      const char* mangled = ti.name();
      if (!mangled)
      {
        throw std::runtime_error("Could get a type name for your type! The world must be ending.");
      }
      boost::mutex::scoped_lock l(mtx);
      std::string rv;
      bool inserted;
      Hashes::iterator it;
      boost::tie(it, inserted) = hashes.insert(std::make_pair<std::string, std::string>(mangled, rv));
      if (inserted)
      {
        {
          int status;
          char* demangled = abi::__cxa_demangle(mangled, 0, 0, &status);
          if (status != 0)
            rv = mangled;
          else
            rv = demangled;
          free(demangled);
        }
        it->second = rv;
        //std::cout << "looking up id for : " << rv << " inserted " << size_t(it->second.c_str()) << std::endl;
      }
      return it->second;
    }
    typedef std::map<std::string, std::string> Hashes;
    boost::mutex mtx;
    Hashes hashes;
  };
  const std::string& name_of(const std::type_info &ti)
  {
    static type_mapping TypeMasterIndex;
    return TypeMasterIndex.lookup(ti);
  }
}

