#include <ecto/all.hpp>

#include <boost/thread.hpp>

namespace ecto {
  namespace graph {

    struct edge::impl {
      std::string from_port, to_port;
      boost::mutex mtx;
      std::deque<ecto::tendril> deque;
    };

    edge::edge(const std::string& fp, const std::string& tp) 
      : impl_(new impl)
    { 
      impl_->from_port = fp;
      impl_->to_port = tp;
    }

    const std::string& edge::from_port() {
      return impl_->from_port;
    }

    const std::string& edge::to_port() {
      return impl_->to_port;
    }

    tendril& edge::front() 
    { 
      boost::unique_lock<boost::mutex> lock(impl_->mtx);
      return impl_->deque.front();
    }

    void edge::pop_front() 
    { 
      boost::unique_lock<boost::mutex> lock(impl_->mtx);
      impl_->deque.pop_front(); 
    }
    void edge::push_back(const ecto::tendril& t) 
    {
      boost::unique_lock<boost::mutex> lock(impl_->mtx);
      impl_->deque.push_back(t);
    }
    std::size_t edge::size() 
    {
      boost::unique_lock<boost::mutex> lock(impl_->mtx);
      return impl_->deque.size(); 
    }

  }
}
