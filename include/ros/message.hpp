#ifndef ROS_MESSAGE_HPP_INCLUDED
#define ROS_MESSAGE_HPP_INCLUDED

#include <ros/allocator.hpp>
#include <boost/noncopyable.hpp>
#include <iostream>

template <typename T>
struct message
{
  struct impl_t {
    unsigned refcount;
    T data;
  };

  impl_t* impl;

  message(impl_t* impl_) 
  {
    SHOW("constructing msg impl == " << impl_);
    impl = impl_;
    ++(impl->refcount);
  }
  message(const message& rhs) 
  {
    impl = rhs.impl;
    ++(impl->refcount);
  }
  message& operator=(const message& rhs) 
  {
    impl = rhs.impl;
    ++(impl->refcount);
  }


  T* operator->() 
  {
    return &(impl->data);
  }

  T& operator*() 
  {
    return impl->data;
  }

  ~message() 
  {
    --(impl->refcount);
  }
  friend std::ostream& operator<<(std::ostream& os, const message::impl_t& m){
    return os << m.data << " refcount=" << m.refcount;
  }
};

#endif
