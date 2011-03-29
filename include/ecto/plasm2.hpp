#pragma once
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <string>

namespace ecto
{
  //forward declare module so we don't get affected by its header
  class module;

  struct plasm2 : boost::noncopyable
  {
    typedef boost::shared_ptr<module> modulePtr;

    plasm2();

    void connect(modulePtr from, const std::string& output, modulePtr to, const std::string& input);
    void markDirty(modulePtr m);
    void go(modulePtr m);
    void viz(std::ostream& out) const;
    std::string viz() const;
    class impl;
    boost::shared_ptr<impl> impl_;
  };
}
