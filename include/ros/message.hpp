#ifndef ROS_MESSAGE_HPP_INCLUDED
#define ROS_MESSAGE_HPP_INCLUDED

#include <ros/allocator.hpp>

template <typename T>
struct message : allocable
{

  unsigned refcount;
  T data;

public:

  T* operator->()
  {
    return &data;
  }

};


#endif
