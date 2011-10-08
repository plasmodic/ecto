#pragma once

#include <boost/shared_ptr.hpp>
namespace ecto {
  class tendril;
  typedef boost::shared_ptr<tendril> tendril_ptr;
  typedef boost::shared_ptr<const tendril> tendril_cptr;

  class tendrils;
  typedef boost::shared_ptr<tendrils> tendrils_ptr;
  typedef boost::shared_ptr<const tendrils> tendrils_cptr;

  struct cell;
  typedef boost::shared_ptr<cell> cell_ptr;
  typedef boost::shared_ptr<const cell> cell_cptr;

  struct plasm;
  typedef boost::shared_ptr<plasm> plasm_ptr;
  typedef boost::shared_ptr<const plasm> plasm_cptr;

  namespace graph {
    struct edge;
    typedef boost::shared_ptr<edge> edge_ptr;
    typedef boost::shared_ptr<const edge> edge_cptr;
  }

  template <typename T> struct cell_;
}
