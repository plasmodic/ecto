#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <ecto/tendril.hpp>
#include <ecto/tendrils.hpp>

#include <map>

namespace ecto 
{
  struct module : boost::noncopyable
  {
    typedef boost::shared_ptr<module> ptr;
    typedef tendrils tendrils_t;
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
         return outputs.set<T>(name,doc,t);
       }

       template<typename T>
       tendril& setIn(const std::string& name,
                      const std::string& doc = "",
                      const T& t = T())
       {
          return inputs.set<T>(name,doc,t);
       }

       template<typename T>
       tendril& setParam(const std::string& name,
                         const std::string& doc = "",
                         const T& t = T())
       {
         return params.set<T>(name,doc,t);
       }


    template<typename T>
    T& getOut(const std::string& name)
    {
      return outputs.get<T>(name);
    }

    template<typename T>
    const T& getIn(const std::string& name) const
    {
      return inputs.get<T>(name);
    }

    template<typename T>
    const T& getParam(const std::string& name) const
    {
      return params.get<T>(name);
    }


    tendrils inputs, outputs, params;
  private:
    bool dirty_;

    friend class plasm2;
  };

}//namespace ecto
