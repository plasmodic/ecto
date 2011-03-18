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
    tendril& setOut(const std::string& name,
		    const std::string& doc = "",
		    const T& t = T())
    {
      outputs[name] = ecto::tendril::make<T>(t,doc);
      return outputs[name];
    }

    template<typename T>
    tendril& setIn(const std::string& name,
		   const std::string& doc = "",
		   const T& t = T())
    {
      inputs[name] = ecto::tendril::make<T>(t,doc);
      return inputs[name];
    }

    template<typename T>
    tendril& setParam(const std::string& name,
		      const std::string& doc = "",
		      const T& t = T())
    {
      params[name] = ecto::tendril::make<T>(t,doc);
      return params[name];
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
