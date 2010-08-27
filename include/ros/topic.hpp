#ifndef ROS_TOPIC_HPP_INCLUDED
#define ROS_TOPIC_HPP_INCLUDED


#include <iostream>
#include <ros/message.hpp>
#include <ros/util.hpp>

template <typename T>
class topic 
{
  
public:

  message<T> create() {
    SHOW("making message");
    return message<T>();
  }

};

#endif
