#pragma once
#include <boost/python.hpp>
#include <boost/python/type_id.hpp>
#include <boost/shared_ptr.hpp>

#include <ecto/util.hpp> //name_of
#include <stdexcept>
#include <string>
#include <set>
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
 *
 * Tendrils also support being tied to other tendrils so that many tendrils
 * may point to the same underlying data. AKA the multi pointer paradigm
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
   * \brief desctructor, will disconnect the tendril.
   */
  ~tendril();
  /**
   * \brief The copy constructor assures that all the tendrils points towards the right value
   * @param rhs the tendril to copy from
   */
  tendril(const tendril& rhs);

  /**
   * \brief The assignment operator will release the tendril from another's grasp, and assure the multi pointer
   * paradigm.
   * @param rhs
   * @return
   */
  tendril& operator=(const tendril& rhs);

  template<typename T>
  tendril(const T& t, const std::string& doc) :
    impl_(new impl<T> (t, this)), connected_(false)
  {
    setDoc(doc);
  }

  /**
   * \brief set this tendril with a doc string and default type. If the types are not the same an exception
   * will be thrown.
   * @param doc docstring of this tendril.
   * @param t a default value.
   */
  template<typename T>
  void set(const std::string& doc, const T& t)
  {
    if (is_type<T> () || is_type<none> ())
    {
      *this = tendril(t, doc); //this will disconnect the tendril to existing tendrils.
    }
    else
      enforce_type<T> (); //throws since the tendril is already a different type.
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

  /**
   * \brief The doc for this tendril is runtime defined, so you may want to update it.
   * @param doc_str A human readable description of the tendril.
   */
  void setDoc(const std::string& doc_str);

  /**
   * Given T this will get the type from the tendril, also enforcing type with an exception.
   * @return a const reference to the value of the tendril (no copies)
   */
  template<typename T>
  inline const T& get() const
  {
    //throws on failure
    enforce_type<T> ();
    //cast a void pointer to this type.
    return *static_cast<const T*> (impl_->get());
  }

  /**
   * Given T this will get the type from the tendril, also enforcing type with an exception.
   * @return a reference to the value of the tendril (no copies)
   */
  template<typename T>
  inline T& get()
  {
    //throws on failure
    enforce_type<T> ();
    //cast a void pointer to this type.
    return *static_cast<T*> (impl_->get());
  }

  /**
   * \brief runtime check if the tendril is of the given type.
   * @return true if it is the type.
   */
  template<typename T>
  inline bool is_type() const
  {
    return impl_base::check<T>(*impl_);
  }

  inline bool same_type(const tendril& rhs) const
  {
    return type_name() == rhs.type_name();
  }

  /**
   * \brief runtime check if the tendril is of the given type, this will throw.
   */
  template<typename T>
  inline void enforce_type() const
  {
    if (!is_type<T> ())
      throw(std::logic_error(
          std::string(type_name() + " is not a " + name_of<T> ()).c_str()));
  }

  /**
   * \brief This ties the tendrils so that they look at the same object. This ensures that all preconnected
   * tendrils point to the same object.  The existing tendril value may be destroyed in favor of the source tendril.
   * If a tendril is of a stronger type it will override the other tendril.
   * @param source  The tendril to connect to.
   */
  void connect(tendril& source);

  /**
   * \brief disconnect this tendril from all tendrils previously associated with it.
   */
  void disconnect();

  /**
   * \brief Get the boost::python version of the object (by value)
   * @return A copy of the underlying object as a boost python object, will be None type if the conversion fails.
   */
  boost::python::object extract() const;
  /**
   * \brief Set this tendril's value from the python object. This will copy the value
   * @param o a python object holding a type compatible with this tendril. Will throw if the types are not compatible.
   */
  void set(boost::python::object o);

  /** Check if this tendril is preconnected.
   * @return true if connected.
   */
  bool connected() const
  {
    return connected_;
  }

  /** A none type for tendril when the tendril is uninitialized.
   */
  struct none
  {
  };

private:
  // ############################### NVI ####################################
  struct impl_base
  {
    typedef boost::shared_ptr<impl_base> ptr;
    impl_base() :
      doc()
    {
    }
    virtual ~impl_base();
    virtual const std::string& type_name() const = 0;
    virtual void* get() = 0;
    virtual bool is_type(std::type_info const& ti) const = 0;

    virtual void setPython(boost::python::object o) = 0;
    virtual boost::python::object getPython() const = 0;
    virtual ptr make(tendril* owner) const = 0;

    //convenience functions for checking types
    template<typename T>
    static bool inline check(impl_base& i);
    template<typename T>
    static inline void checkThrow(impl_base& i) throw (std::logic_error);
    std::string doc;
    std::set<tendril*> owners;
  };

  template<typename T>
  struct impl: impl_base
  {
    impl(const T& t, tendril* owner);
    const std::string& type_name() const;
    bool is_type(std::type_info const& ti) const;
    void* get();
    void setPython(boost::python::object o);
    boost::python::object getPython() const;
    impl_base::ptr make(tendril* owner) const
    {
      impl_base::ptr p(new impl<T> (t, owner));
      p->doc = doc;
      return p;
    }
    T t;
  };

  tendril(impl_base::ptr impl);
  boost::shared_ptr<impl_base> impl_;
  bool connected_;
};

template<typename T>
bool tendril::impl_base::check(tendril::impl_base& i)
{
  return i.is_type(typeid(T));
}

template<typename T>
void tendril::impl_base::checkThrow(tendril::impl_base& i)
    throw (std::logic_error)
{
  if (!check<T> (i))
    throw(std::logic_error(
        std::string(i.type_name() + " is not a " + name_of<T> ()).c_str()));
}

template<typename T>
tendril::impl<T>::impl(const T& t, tendril * const owner) :
  t(t)
{
  if (owner != NULL)
    owners.insert(owner);
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
  return &t;//.get();
}

template<typename T>
void tendril::impl<T>::setPython(boost::python::object o)
{
  boost::python::extract<T> get_T(o);
  if (get_T.check())
    t = get_T();
  else
    throw std::logic_error(
        "Could not convert python object to type : " + type_name());
}

template<typename T>
boost::python::object tendril::impl<T>::getPython() const
{
  try
  {
    boost::python::object o(t);
    return o;
  } catch (const boost::python::error_already_set&)
  {
    //silently handle no python wrapping
  }
  return boost::python::object();
}

}
