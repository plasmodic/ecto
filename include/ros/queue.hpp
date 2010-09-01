#ifndef ROS_TOPIC_HPP_INCLUDED
#define ROS_TOPIC_HPP_INCLUDED

#include <iostream>
#include <ros/message.hpp>
#include <ros/util.hpp>
#include <ros/allocator.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

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
  typedef typename message_t::impl_t message_impl_t;
  message_impl_t* data;

  
  friend std::ostream& operator<<(std::ostream& os, const queue& q) 
  {
    os << "[Queue: header @ " << q.header << "\n"
       << "        data   @ " << q.data << "\n"
       << "        length = " << q.header->length << "\n"
       << "        head   = " << q.header->head_index << "\n"
       << "\n";

    for (unsigned i=0, end=q.header->length; 
	 i < end;
	 ++i) 
      {
	os << q.data[i];
	if (i == q.header->head_index)
	  os << " <---------------------";
	os << "\n";
      }
    os << "\n";
    return os;
  }

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

    data = static_cast<message_impl_t*>(data_region.get_address());

    for(unsigned i = 0; i<size; ++i)
      {
	new (data+i) message_impl_t;
	data[i].refcount = 0;
      }
    SHOW("region = " << data);

  }

  message_t create() 
  {
    SHOW("making message at "<< header->head_index);
    
    bip::scoped_lock<bip::interprocess_mutex> lock(header->mutex);
    unsigned thishead = header->head_index;
    data[thishead].~message_impl_t();
    new (data+thishead) message_impl_t;
    SHOW("newing at " << data+thishead);
    header->head_index = (thishead + 1) % header->length;
    
    message_t msg(data + thishead);
    return msg;
  }
};

#endif
