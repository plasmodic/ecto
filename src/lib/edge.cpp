#include <ecto/edge.hpp>
#include <boost/thread.hpp>

namespace ecto {
  namespace graph {

    edge::edge(const std::string& fp, const std::string& tp) 
      : from_port(fp), to_port(tp)
    { }

    tendril& edge::front() 
    { 
      boost::unique_lock<boost::mutex> lock(mtx);
      return deque.front();
    }

    void edge::pop_front() 
    { 
      boost::unique_lock<boost::mutex> lock(mtx);
      deque.pop_front(); 
    }
    void edge::push_back(const ecto::tendril& t) 
    {
      boost::unique_lock<boost::mutex> lock(mtx);
      deque.push_back(t);
    }
    std::size_t edge::size() 
    {
      boost::unique_lock<boost::mutex> lock(mtx);
      return deque.size(); 
    }

  }
}
