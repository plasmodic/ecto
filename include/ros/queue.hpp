#ifndef ROS_TOPIC_HPP_INCLUDED
#define ROS_TOPIC_HPP_INCLUDED

#include <iostream>
#include <ros/message.hpp>
#include <ros/util.hpp>
#include <ros/allocator.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

namespace bip = boost::interprocess;

template <typename T>
class queue 
{
  std::string name;
  
  bip::shared_memory_object shm;
  bip::mapped_region header_region, data_region;

  struct header_t
  {
    bip::interprocess_mutex mutex;
    unsigned head_index;
    unsigned length;
  };
  header_t* header;
  
  typedef message<T> message_t;
  message_t* data;

public:

  queue(const std::string& name_, std::size_t size, bip::mode_t mode)
    : name(name_)
    , shm(bip::open_or_create, name.c_str(), mode)
  {
    shm.truncate(sizeof(header_t) + size * sizeof(message_t));

    bip::mapped_region(shm, bip::read_write,
		       0, sizeof(header_t))
      .swap(header_region);

    header = static_cast<header_t*>(header_region.get_address());
    header->length = size;
    header->head_index = 0;

    bip::mapped_region(shm, bip::read_write,
		       sizeof(header_t), size * sizeof(message_t))
      .swap(data_region);

    data = static_cast<message_t*>(data_region.get_address());

    for(unsigned i = 0; i<size; ++i)
      {
	new (data+i) message_t;
      }
    SHOW("region = " << data);

  }

  message_t& create() 
  {
    SHOW("making message at "<< header->head_index);
    
    unsigned thishead = header->head_index;
    data[thishead].~message_t();
    new (data+thishead) message_t;
    header->head_index = (thishead + 1) % header->length;
    return data[thishead];
  }
};

#endif
