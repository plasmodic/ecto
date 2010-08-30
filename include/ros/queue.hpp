#ifndef ROS_TOPIC_HPP_INCLUDED
#define ROS_TOPIC_HPP_INCLUDED

#include <iostream>
#include <ros/message.hpp>
#include <ros/util.hpp>
#include <ros/allocator.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

namespace bip = boost::interprocess;

template <typename T>
class queue 
{
  std::string name;
  unsigned size;
  message<T>* data;
  bip::shared_memory_object shm;
  bip::mapped_region reg;

  unsigned head;

public:

  typedef message<T> message_t;

  queue(const std::string& name_, std::size_t size_, bip::mode_t mode)
    : name(name_)
    , size(size_)
    , shm(bip::open_or_create, name.c_str(), mode)
  {
    head = 0;
    shm.truncate(size * sizeof(message_t));

    bip::mapped_region(shm, bip::read_write,
		       0, size * sizeof(message_t))
      .swap(reg);

    data = static_cast<message_t*>(reg.get_address());

    SHOW("region = " << data);

  }

  message_t& create() 
  {
    SHOW("making message at "<< head);
    
    unsigned thishead = head;
    new (data+thishead) message_t;
    head = (head + 1) % size;
    return data[thishead];
  }
};

#endif
