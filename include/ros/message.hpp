#ifndef ROS_MESSAGE_HPP_INCLUDED
#define ROS_MESSAGE_HPP_INCLUDED

#include <ros/allocator.hpp>
#include <boost/noncopyable.hpp>
#include <iostream>

template <typename T>
struct message_ptr
{
  struct impl_t 
  {
    unsigned refcount;
    T data;
  };

  impl_t* impl;

  static std::size_t shm_size() { return sizeof(impl_t); }

  message_ptr(impl_t* impl_) 
  {
    SHOW("constructing msg impl == " << impl_);
    impl = impl_;
    ++(impl->refcount);
  }
  message_ptr(const message_ptr& rhs) 
  {
    impl = rhs.impl;
    ++(impl->refcount);
  }
  message_ptr& operator=(const message_ptr& rhs) 
  {
    impl = rhs.impl;
    ++(impl->refcount);
    return *this;
  }

  T* operator->() 
  {
    return &(impl->data);
  }

  T& operator*() 
  {
    return impl->data;
  }

  ~message_ptr() 
  {
    --(impl->refcount);
  }
  friend std::ostream& operator<<(std::ostream& os, const message_ptr::impl_t& m){
    return os << m.data << " refcount=" << m.refcount;
  }
};

#endif
