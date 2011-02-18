#include <ecto/util.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/smart_ptr/intrusive_ptr.hpp>

using namespace boost::interprocess;

namespace N 
{
  //A class that has an internal reference count
  template <typename T>
  class rc_base
  {
  public:

    typedef managed_shared_memory::segment_manager segment_manager;
    typedef intrusive_ptr<T, offset_ptr<void> > ptr;

  private:
    //Non-copyable
    rc_base(const rc_base  &);
    //Non-assignable
    rc_base & operator=(const rc_base &);
    //A typedef to save typing
    //This is the reference count
    unsigned int m_use_count;
    //The segment manager allows deletion from shared memory segment
    offset_ptr<segment_manager> mp_segment_manager;

  public:
    //Constructor
    rc_base(segment_manager *s_mngr)
      : m_use_count(0), mp_segment_manager(s_mngr){}
    //Destructor
    ~rc_base(){}

  public:
    //Returns the reference count
    unsigned int use_count() const
    {  return m_use_count;   }

    //Adds a reference
    inline friend void intrusive_ptr_add_ref(T * p)
    {  
      SHOW(p);
      ++p->m_use_count; 
    }

    //Releases a reference
    inline friend void intrusive_ptr_release(T * p)
    {  
      SHOW(p);
      if(--p->m_use_count == 0)  
	p->mp_segment_manager->destroy_ptr(p); 
    }

  };

  
  struct Point : rc_base<Point> 
  {
    float x, y, z;
    Point(segment_manager *s_mngr) : rc_base<Point>(s_mngr)
    {
      x = y = z = 0.0;
    }
  };

}

//A class that has an intrusive pointer to rc_base
class intrusive_ptr_owner
{
  N::Point::ptr m_intrusive_ptr;

public:
  //Takes a pointer to the reference counted class
  intrusive_ptr_owner(N::Point *ptr) 
    : m_intrusive_ptr(ptr){}
};

template <typename T>
struct shm_queue
{
  managed_shared_memory shmem;

  shm_queue(const std::string& name, unsigned size)
    : shmem(create_only, name, size * sizeof(T));
  }

  T::ptr

  
};

int main()
{
  //Remove shared memory on construction and destruction
  struct shm_remove
  {
    shm_remove() { shared_memory_object::remove("MySharedMemory"); }
    ~shm_remove(){ shared_memory_object::remove("MySharedMemory"); }
  } remover;

  //Create shared memory
  managed_shared_memory shmem(create_only, "MySharedMemory", 10000);

  //Create the unique reference counted object in shared memory
  N::Point *ref_counted = shmem.construct<N::Point>("ref_counted")(shmem.get_segment_manager());

  //Create an array of ten intrusive pointer owners in shared memory
  intrusive_ptr_owner *intrusive_owner_array = 
    shmem.construct<intrusive_ptr_owner>(anonymous_instance)[10](ref_counted);

  //Now test that reference count is ten
  if(ref_counted->use_count() != 10)
    return 1;

  //Now destroy the array of intrusive pointer owners
  //This should destroy every intrusive_ptr and because of
  //that rc_base will be destroyed
  shmem.destroy_ptr(intrusive_owner_array);

  //Now the reference counted object should have been destroyed
  if(shmem.find<intrusive_ptr_owner>("ref_counted").first)
    return 1;
  //Success!
  return 0;
}
