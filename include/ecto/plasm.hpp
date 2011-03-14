#pragma once
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <string>
#include <map>
#include <set>

namespace ecto
{
  //forward declare module so we don't get affected by its header
  class module;

  namespace plasm_ops
  {
    template<typename T>
      struct oless
      {
        bool operator()(const T& lhs, const T& rhs)
        {
          return lhs.get() < rhs.get();
        }
      };
    typedef oless<boost::shared_ptr<module> > lessMPtr;
  }

  /** \brief A directed graph per node edge description,
   *         with a notion of upstream and downstream edges
   */
  struct edge
  {
    struct ds
    {
      typedef std::set<boost::shared_ptr<module>, plasm_ops::lessMPtr> module_set;
      typedef std::map<std::string, module_set> modules;
      typedef modules::iterator iterator;
      typedef modules::const_iterator const_iterator;
    };
    struct us
    {
      typedef std::map<std::string, boost::shared_ptr<module> > modules;
      typedef modules::iterator iterator;
      typedef modules::const_iterator const_iterator;
    };

    ds::modules downstream;
    /** Upstream edges, that point from the inputs to the modules
     */
    us::modules upstream;
  };

  struct plasm : boost::noncopyable
  {
    typedef std::map<boost::shared_ptr<module>, edge, plasm_ops::lessMPtr> map_t;
    map_t edge_map;
    void connect(boost::shared_ptr<module> from, const std::string& output, boost::shared_ptr<module> to,
                 const std::string& input);
    void markDirty(boost::shared_ptr<module> m);
    void go(boost::shared_ptr<module> m);
    std::string viz() const;
  };
}
