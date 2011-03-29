#pragma once
#include <ecto/util.hpp> //name_of
#include <boost/shared_ptr.hpp>
#include <boost/python.hpp>

#include <stdexcept>
#include <string>
#include <sstream>
#include <cstring>

namespace ecto
{
  /** \brief A tendril is the slender, winding organ of the
   * ecto::module that gives it its awesome type erasure and uber
   * flexibility.
   *
   * Each tendril is a type erasing holder for any instance of any type,
   * and allows introspection including its value, type, and doc string.
   */
  class tendril
  {
  public:
    /** \brief default constructor, creates a tendril that is not
     * initialized with any value FIXME make this default to a
     * tendril::none type.
     */
    tendril();

    /**  \brief This factory function creates a tendril, holding type T.
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


    /** \brief This is an unmangled type name for what ever tendril is
     * holding.

     * @return the unmangled name, e.g. "cv::Mat", or
     * "pcl::PointCloud<pcl::PointXYZ>"
     */
    std::string type_name() const;

    /** \brief A doc string for this tendril, "foo is for the input
	and will be mashed with spam."

	* @return A very descriptive human readable string of whatever
	* the tendril is holding on to.
	*/
    std::string doc() const;

    template <typename T>
    const T& get() const
    {
      if (!impl_)
	throw std::logic_error("This connection is uninitialized! type: " + name_of<T> ());
      return *impl_base::get<T>(*impl_);
    }

    template <typename T>
    T& get()
    {
      if (!impl_)
	throw std::logic_error("This connection is uninitialized! type: " + name_of<T> ());
      return *(impl_base::get<T>(*impl_));
    }
    template <typename T>
    bool is_type()
    {
      bool same_type = std::strcmp(impl_->type_info().name(), typeid(T).name()) == 0;
      return same_type;
    }
    void connect(tendril& rhs);

    inline bool dirty(bool b)
    {
      dirty_ = b;
      return dirty_;
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
      virtual std::string type_name() const = 0;
      virtual void* get() = 0;
      virtual const std::type_info & type_info() const = 0;
      virtual void setPython(boost::python::object o) = 0;
      virtual boost::python::object getPython() const = 0;
      //convience functions for checking types
      template <typename T>
      static bool inline check(impl_base& i);
      template <typename T>
      static inline void checkThrow(impl_base& i) throw (std::logic_error);
      template <typename T>
      static inline T* get(impl_base& i);

      std::string name, doc;
    };

    template <typename T>
    struct impl : impl_base
    {
      impl(const T& t);
      std::string type_name() const;
      const std::type_info & type_info() const;
      void* get();
      void setPython(boost::python::object o);
      boost::python::object getPython() const;
      T t;
    };
    tendril(impl_base::ptr impl);
    boost::shared_ptr<impl_base> impl_;
    bool dirty_;
    friend class tendrils;
    friend class module;
  };
#include <ecto/impl/tendril.hpp>

}
