#pragma once
#include <ecto/plasm.hpp>
#include <ecto/tendril.hpp>
#include <ecto/module.hpp>
#include <ecto/graph_types.hpp>

#include <string>
#include <map>
#include <set>
#include <utility>
#include <deque>



namespace ecto {

  namespace scheduler {
    
    struct threadpool 
    {
      threadpool(plasm&);

      int execute(unsigned nthreads);

      ecto::graph::graph_t& graph;
      
      struct impl;
      boost::shared_ptr<impl> impl_;

    };
  }
}
