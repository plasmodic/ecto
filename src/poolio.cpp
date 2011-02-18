#include <map>
#include <ecto/util.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/node_allocator.hpp>
#include <boost/interprocess/smart_ptr/intrusive_ptr.hpp>

namespace ip = boost::interprocess;

struct Point {
  float x, y, z;
  unsigned refcount;
  Point() { x=0; y=0; z=0; refcount=0; }
};

void intrusive_ptr_add_ref(Point* p) { ++(p->refcount); } 
void intrusive_ptr_release(Point* p) { --(p->refcount); } 

template <typename Alloc>
struct node_deleter 
{
  Alloc& alloc;
  node_deleter(Alloc& a) : alloc(a) { }
  node_deleter(const node_deleter& rhs) 
    : alloc(rhs.alloc)
  { }
    
  void operator()(typename Alloc::value_type *p)
  {
    alloc.deallocate_one(p);
  }
};

/*
template <typename T>
struct pool_deleter 
{
  boost::object_pool<T>& pool;
  pool_deleter(boost::object_pool<T>& bp) : pool(bp) { }
  pool_deleter(const pool_deleter& rhs) 
    : pool(rhs.pool)
  { }
    
  void operator()(T *p)
  {
    pool.destroy(p);
  }
};

struct shmem_allocator
{
  typedef std::size_t size_type;
  typedef std::ptrdiff_t difference_type;

  static char * malloc(const size_type bytes)
  { 
    return new (std::nothrow) char[bytes]; 
  }
  static void free(char * const block)
  { 
    delete [] block; 
  }
};
*/

int main(int argc, char** argv)
{
  ip::shared_memory_object::remove("shmemmy");
  ip::managed_shared_memory segment(ip::create_only, 
                                    "shmemmy",  //segment name
                                    10000 * sizeof(Point));

  typedef ip::node_allocator<Point, ip::managed_shared_memory::segment_manager>
    node_allocator_t;

  node_allocator_t alloc_instance(segment.get_segment_manager());
  node_deleter<node_allocator_t> dealloc_instance(alloc_instance);

  boost::object_pool<Point> p(100);

  for (int j=0; j<1000; j++)
    {
      boost::shared_ptr<Point> pointptr(alloc_instance.allocate_one(), 
                                        dealloc_instance);
    }
  


}
