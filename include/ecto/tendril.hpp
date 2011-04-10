#pragma once
#include <boost/python.hpp>
#include <boost/python/type_id.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/util.hpp> //name_of

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
    static inline tendril
    make(const T& t = T(), const std::string& doc = std::string())
    {
      // fixme: allocators
      tendril c(impl_base::ptr(new impl<T> (t)));
      c.setDoc(doc);
      return c;
    }

    /**
     *
     * @param doc
     * @param t
     */
    template<typename T>
    inline void set(const std::string& doc, const T& t)
    {
      if(is_type<T>())
      {
        get<T>() = t;
        setDoc(doc);
      }else if(is_type<none>())
      {
        *this = make<T>(t,doc);
      }else
        enforce_type<T>();
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

    void setDoc(const std::string& doc_str);

    template<typename T>
    inline const T& get() const
    {
      enforce_type<T> ();
      return *reinterpret_cast<const T*> (impl_->get());
    }

    template<typename T>
    inline T& get()
    {
      enforce_type<T> ();
      return *reinterpret_cast<T*> (impl_->get());
    }

    template <typename T>
    inline bool is_type() const
    {
      return impl_base::check<T>(*impl_);
    }

    template<typename T>
      inline void enforce_type() const
      {
        if (!is_type<T> ())
          throw(std::logic_error(std::string(type_name() + " is not a " + name_of<T> ()).c_str()));
      }
    void connect(tendril& rhs);

    boost::python::object extract() const;
    void set(boost::python::object o);

    bool connected() const{return connected_;}

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
      virtual bool steal( tendril& to) = 0;
      //convenience functions for checking types
      template <typename T>
      static bool inline check(impl_base& i);
      template <typename T>
      static inline void checkThrow(impl_base& i) throw (std::logic_error);
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
      bool steal(tendril& to);
      T t;
    };

    tendril(impl_base::ptr impl);
    boost::shared_ptr<impl_base> impl_;
    bool connected_;
    friend class tendrils;
    friend class module;
  };
  
template<typename T>
bool tendril::impl_base::check(tendril::impl_base& i)
{
  return i.is_type(typeid(T));
}

template<typename T>
void tendril::impl_base::checkThrow(tendril::impl_base& i) throw (std::logic_error)
{
  if (!check<T> (i))
    throw(std::logic_error(std::string(i.type_name() + " is not a " + name_of<T> ()).c_str()));
}



template<typename T>
tendril::impl<T>::impl(const T& t) :
  t(t)
{
}

template<typename T>
const std::string& tendril::impl<T>::type_name() const
{
  static const std::string name = name_of<T> ();
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
  catch (const boost::python::error_already_set&) {
    //silently handle no python wrapping
  }
  return boost::python::object();
}

template<typename T>
bool tendril::impl<T>::steal(tendril& to)
{
  boost::python::extract<T> get_T(to.extract());
  if (get_T.check())
  {
    std::cout << "stealing to a native tendril..."<< std::endl;
    //reset the impl from a python type to a c++ type.
    typedef impl<T> impl_T;
    impl_T* i_t(new impl_T(get_T()));
    i_t->doc = to.doc();
    to.impl_.reset(i_t);
    return true;
  }
  return false;
}

}
