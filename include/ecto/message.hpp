#ifndef ECTO_MESSAGE_HPP_INCLUDED
#define ECTO_MESSAGE_HPP_INCLUDED

#include <ecto/allocator.hpp>
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

  static message_ptr create_inplace(void* p) 
  {
    impl_t* impl = new (p) impl_t;
    return message_ptr(impl);
  }

  explicit message_ptr(impl_t* p) 
  {
    SHOW("constructing msg impl == " << p);
    impl = p;
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
