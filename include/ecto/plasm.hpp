#pragma once
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <string>

namespace ecto
{
  //forward declare module so we don't get affected by its header
  class module;

  class plasm : boost::noncopyable
  {
  public:
    plasm();
    void connect( boost::shared_ptr<module> from, const std::string& output,  boost::shared_ptr<module> to, const std::string& input);
    void markDirty( boost::shared_ptr<module> m);
    void go( boost::shared_ptr<module> m);

    void viz(std::ostream& out) const;
    std::string viz() const;
  private:
    class impl;
    boost::shared_ptr<impl> impl_;
  };
}
