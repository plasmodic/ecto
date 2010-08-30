#ifndef ROS_MESSAGE_HPP_INCLUDED
#define ROS_MESSAGE_HPP_INCLUDED

#include <ros/allocator.hpp>

template <typename T>
struct message 
{

  unsigned refcount;
  T data;

};

#endif
