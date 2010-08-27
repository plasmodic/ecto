#ifndef ROS_MESSAGE_HPP_INCLUDED
#define ROS_MESSAGE_HPP_INCLUDED


template <typename T>
struct message 
{

  unsigned refcount;
  T* px;

public:

  T* operator->()
  {
    return px;
  }

};


#endif
