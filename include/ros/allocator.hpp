#ifndef ROS_ALLOCATOR_HPP_INCLUDED
#define ROS_ALLOCATOR_HPP_INCLUDED

#include <ros/util.hpp>

struct shared_allocator {

  void* alloc(std::size_t s) throw (std::bad_alloc)
  {
    return ::operator new(s);
  }

  void dealloc(void* p) {
    ::operator delete(p);
  }
};


struct allocable {

  void* operator new(std::size_t s, shared_allocator& a)
  {
    SHOW(s);
    return a.alloc(s);
  }

  void operator delete(void* p, shared_allocator& a)
  {
    SHOW(p);
    a.dealloc(p);
  }

};


#endif

