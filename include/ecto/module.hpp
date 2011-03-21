#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <ecto/tendril.hpp>

#include <map>

namespace ecto 
{
  struct module : boost::noncopyable
  {
    typedef boost::shared_ptr<module> ptr;
    typedef std::map<std::string, tendril> tendrils_t;
    module();
    virtual ~module();
    virtual void Process();
    virtual void Config();

    void connect(const std::string& output, ptr to, const std::string& input);
    void dirty(bool hmm);
    bool dirty() const;

    template<typename T>
    inline static tendril& set(tendrils_t& tendrils,
                               const std::string& name,
                               const std::string& doc,
                               const T& t)
    {
      //if there are no exiting tendrils by the given name,
      //just add it.
      if (tendrils.count(name) == 0)
      {
        tendrils[name] = ecto::tendril::make<T>(t, doc);
      }else // we want to just return the existing tendril (so that modules preconnected don't get messed up)...
      {
        //there is already an existing tendril with the given name
        //check if the types are the same
        bool same_type = std::strcmp(tendrils[name].impl_->type_info().name(), typeid(T).name()) == 0;
        if(!same_type)
        {
          //to throw or not to throw...
          std::stringstream ss;
          ss << "Your types aren't the same, this could lead to very undefined behavior...";
          ss << " old type = " << tendrils[name].impl_->type_info().name()
             << " new type = " <<  typeid(T).name() << std::endl;
          throw std::logic_error(ss.str());
          //tendrils[name] = ecto::tendril::make<T>(t, doc);
        }
      }
      return tendrils[name];
    }

    template<typename T>
    tendril& setOut(const std::string& name,
		    const std::string& doc = "",
		    const T& t = T())
    {
      return set<T>(outputs,name,doc,t);
    }

    template<typename T>
    tendril& setIn(const std::string& name,
		   const std::string& doc = "",
		   const T& t = T())
    {
      return set<T>(inputs,name,doc,t);
    }

    template<typename T>
    tendril& setParam(const std::string& name,
		      const std::string& doc = "",
		      const T& t = T())
    {
      return set<T>(params,name,doc,t);
    }

    template<typename T>
    T& getOut(const std::string& name)
    {
      //FIXME check for null, throw
      return outputs[name].get<T>();
    }

    template<typename T>
    const T& getIn(const std::string& name)
    {
      //FIXME check for null, throw
      return inputs[name].get<T>();
    }

    template<typename T>
    const T& getParam(const std::string& name)
    {
      //FIXME check for null, throw
      return params[name].get<T>();
    }


    tendrils_t inputs, outputs, params;
    //std::string doc,name;
  private:
    bool dirty_;

    friend class plasm;
  };

}//namespace ecto
