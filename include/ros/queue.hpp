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

namespace ip = boost::interprocess;

template <typename T>
class queue 
{
  std::string name;
  
  ip::shared_memory_object shm;
  ip::mapped_region header_region, data_region;

  struct item {
    T data;
    unsigned refcount;
  };

  struct header_t
  {
    ip::interprocess_mutex mutex;
    unsigned head_index;
    unsigned length;
  };
  header_t* header;
  
  typedef message_ptr<T> message_ptr_t;
  typedef typename message_ptr_t::impl_t message_ptr_impl_t;
  message_ptr_impl_t* data;

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

  queue(const std::string& name_, std::size_t size, ip::mode_t mode)
    : name(name_)
    , shm(ip::open_or_create, name.c_str(), mode)
  {

    shm.truncate(sizeof(header_t) + size * message_ptr_t::shm_size());

    ip::mapped_region(shm, ip::read_write,
		       0, sizeof(header_t))
      .swap(header_region);

    header = static_cast<header_t*>(header_region.get_address());
    header->length = size;
    header->head_index = 0;

    ip::mapped_region(shm, ip::read_write,
		       sizeof(header_t), size * message_ptr_t::shm_size())
      .swap(data_region);

    data = static_cast<message_ptr_impl_t*>(data_region.get_address());

    for(unsigned i = 0; i<size; ++i)
      {
	message_ptr_t mp(data+i);
      }
    SHOW("region = " << data);

  }

  message_ptr_t create() 
  {
    SHOW("making message at "<< header->head_index);
    
    ip::scoped_lock<ip::interprocess_mutex> lock(header->mutex);
    unsigned thishead = header->head_index;
    data[thishead].~message_ptr_impl_t();
    message_ptr_t newptr = message_ptr_t::create_inplace(data + thishead);

    // new (data+thishead) message_ptr_impl_t;
    // SHOW("newing at " << data+thishead);
    header->head_index = (thishead + 1) % header->length;
    
    return newptr;
  }
};

#endif
