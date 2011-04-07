#pragma once
#include <ecto/util.hpp> //name_of
#include <boost/shared_ptr.hpp>
#include <boost/python.hpp>
#include <boost/python/type_id.hpp>
#include <stdexcept>
#include <string>
#include <sstream>
#include <cstring>

namespace ecto
{
  /**
   * \brief A tendril is the slender, winding organ of the
   * ecto::module that gives it its awesome type erasure and uber
   * flexibility.
   *
   * Each tendril is a type erasing holder for any instance of any type,
   * and allows introspection including its value, type, and doc string.
   */
  class tendril
  {
  public:
    /**
     * \brief default constructor, creates a tendril that is initialized with the
     * tendril::none type.
     */
    tendril();

    /**
     * \brief This factory function creates a tendril, holding type T.
     *
     * @param t the default value of t
     * @param name the instance name of t
     * @param doc docstring for t
     * @return a tendril holding a copy of t
     */
    template <typename T>
    static tendril 
    make(const T& t = T(), const std::string& doc = std::string())
    {
      // fixme: allocators
      tendril c(impl_base::ptr(new impl<T> (t)));
      c.impl_->doc = doc;
      return c;
    }



    boost::python::object extract();
    void set(boost::python::object o);

    template<typename T>
    void set(const std::string& doc, const T& t)
    {
      *this = make<T>(t,doc);
    }


    /**
     * \brief This is an unmangled type name for what ever tendril is
     * holding.
     *
     * @return the unmangled name, e.g. "cv::Mat", or
     * "pcl::PointCloud<pcl::PointXYZ>"
     */
    std::string type_name() const;

    /**
    * \brief A doc string for this tendril, "foo is for the input
    * and will be mashed with spam."
    * @return A very descriptive human readable string of whatever
    * the tendril is holding on to.
    */
    std::string doc() const;
    void setDoc(const std::string& doc_str)
    {
      impl_->doc = doc_str;
    }

    template <typename T>
    const T& get() const
    {
      return *impl_base::get<T>(*impl_);
    }

    template <typename T>
    T& get()
    {
      return *(impl_base::get<T>(*impl_));
    }
    template <typename T>
    bool is_type()
    {
      return impl_base::check<T>(*impl_);
    }
    void connect(tendril& rhs);

    inline void dirty(bool b)
    {
      dirty_ = b;
    }

    inline bool dirty() const
    {
      return dirty_;
    }

    struct none
    {
    };

  private:
    // ############################### NVI ####################################
    struct impl_base
    {
      typedef boost::shared_ptr<impl_base> ptr;
      virtual ~impl_base();
      virtual const std::string& type_name() const = 0;
      virtual void* get() = 0;
      virtual bool is_type(std::type_info const& ti) const = 0;
      virtual void setPython(boost::python::object o) = 0;
      virtual boost::python::object getPython() const = 0;
      //convience functions for checking types
      template <typename T>
      static bool inline check(impl_base& i);
      template <typename T>
      static inline void checkThrow(impl_base& i) throw (std::logic_error);
      template <typename T>
      static inline T* get(impl_base& i);

      std::string doc;
    };

    template <typename T>
    struct impl : impl_base
    {
      impl(const T& t);
      const std::string& type_name() const;
      bool is_type(std::type_info const& ti) const;
      void* get();
      void setPython(boost::python::object o);
      boost::python::object getPython() const;
      T t;
    };

      struct impl_py : impl_base
      {
        typedef boost::python::object T;
        impl_py(const T& t) :
          t(t)
        {

        }
        const std::string& type_name() const
        {
          type_name_ ="boost::python::object";
          return type_name_;
        }

        bool is_type(std::type_info const& ti) const
        {
          return true;
        }
        void* get()
        {
          return NULL;
        }
        void setPython(boost::python::object o)
        {
          t = o;
        }
        boost::python::object getPython() const
        {
          return t;
        }
        T t;
        mutable std::string type_name_;
      };
    tendril(impl_base::ptr impl);
    boost::shared_ptr<impl_base> impl_;
    bool dirty_;
    friend class tendrils;
    friend class module;
  };
  
//not sure if we should disable this even in release...
//#if NDEBUG
#if 0
#define ECTO_ASSERT(_impl_check_ )        \
  do {} while(false)
#else
#define ECTO_ASSERT(_impl_check_ )\
  do\
    {\
      _impl_check_;\
    }while(false)
#endif
template<typename T>
bool tendril::impl_base::check(tendril::impl_base& i)
{
  //this fails across shared library boundaries
  //return typeid(T) == i.type_info();
  //however type name should be ok...
  return i.is_type(typeid(T));
}

template<typename T>
void tendril::impl_base::checkThrow(tendril::impl_base& i) throw (std::logic_error)
{
  if (!check<T> (i))
    throw(std::logic_error(std::string(i.type_name() + " is not a " + name_of<T> ()).c_str()));
}

template<typename T>
T* tendril::impl_base::get(tendril::impl_base& i)
{
  ECTO_ASSERT(checkThrow<T>(i));
  return reinterpret_cast<T*> (i.get());
}
template<typename T>
tendril::impl<T>::impl(const T& t) :
  t(t)
{
}

template<typename T>
const std::string& tendril::impl<T>::type_name() const
{
  static const std::string name =  name_of<T> ();
  return name;
}
template<typename T>
bool tendril::impl<T>::is_type(std::type_info const& ti) const
{
  return std::strcmp(typeid(T).name(), ti.name()) == 0;
}

template<typename T>
void* tendril::impl<T>::get()
{
  return &t;
}

template<typename T>
void tendril::impl<T>:: setPython(boost::python::object o)
{
  boost::python::extract<T> get_T(o);
  if(get_T.check())
    t = get_T();
  else
    throw std::logic_error("Could not convert python object to type : " + type_name());
}

template<typename T>
boost::python::object tendril::impl<T>::getPython() const
{
  try
  {
    boost::python::object o(t);
    return o;
  }
  catch (const boost::python::error_already_set& e) {
    //silently handle no python wrapping
  }
  return boost::python::object();
}

template<>
inline tendril
tendril::make<boost::python::object>(const boost::python::object& t, const std::string& doc)
{
  // fixme: allocators
  tendril c(impl_base::ptr(new impl_py(t)));
  c.impl_->doc = doc;
  return c;
}

}
