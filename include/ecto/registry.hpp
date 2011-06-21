#pragma once
#include <boost/noncopyable.hpp>

namespace ecto {
  namespace registry {

    template <typename T>
    struct module_registry : boost::noncopyable
    {
      typedef boost::function<void(void)> nullary_fn_t;

      void add(nullary_fn_t f)
      {
        regvec.push_back(f);
      }

      void go()
      {
        for (unsigned j=0; j<regvec.size(); ++j)
          regvec[j]();
      }

      std::vector<nullary_fn_t> regvec;

      static module_registry& instance()
      {
        static module_registry instance_;
        return instance_;
      }

    private:

      module_registry() { }
    };

    template <typename Module, typename T>
    struct registrator {
      const char* name_;
      const char* docstring_;

      explicit registrator(const char* name, const char* docstring) 
        : name_(name), docstring_(docstring) 
      { 
        module_registry<Module>::instance().add(boost::ref(*this));
      }

      void operator()() const 
      {
        ecto::wrap<T>(name_, docstring_);
      }
      const static registrator& inst;

    };
  }
}

//  namespace ecto { namespace tag { struct MODULE; } }                 

#define ECTO_MODULETAG(MODULE) namespace ecto { namespace tag { struct MODULE; } }

#define ECTO_MODULE(MODULE, TYPE, NAME, DOCSTRING)                      \
  ECTO_MODULETAG(MODULE)                                                \
  namespace {                                                           \
    template<>                                                          \
    const ::ecto::registry::registrator< ::ecto::tag::MODULE,TYPE>&     \
    ::ecto::registry::registrator< ::ecto::tag::MODULE,TYPE>::inst      \
    (::ecto::registry::registrator< ::ecto::tag::MODULE,TYPE>(NAME, DOCSTRING)); \
  }
  
#define ECTO_INSTANTIATE_REGISTRY(MODULE)                               \
  ECTO_MODULETAG(MODULE)                                                \
  template class ::ecto::registry::module_registry< ::ecto::tag::MODULE>;

#define ECTO_REGISTER(MODULE)                                           \
    ::ecto::registry::module_registry< ::ecto::tag::MODULE>::instance().go();
