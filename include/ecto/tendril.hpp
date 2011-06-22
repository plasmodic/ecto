/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include <boost/python.hpp>
#include <boost/python/type_id.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

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
   */
  class tendril
  {
  public:
    typedef boost::shared_ptr<tendril> ptr;
    typedef boost::shared_ptr<const tendril> const_ptr;
    /**
     * \brief default constructor, creates a tendril that is initialized with the
     * tendril::none type.
     */
    tendril();
    /**
     * \brief destructor, will deallocate the value held.
     */
    ~tendril();


    /**
     * \brief A templated convenience constructor for creating a tendril
     * that hides the given type.
     * @tparam T The type to hide in this tendril
     * @param t default value for t
     * @param doc a documentation string
     */
    template<typename T>
    tendril(const T& t, const std::string& doc) :
      holder_(new holder<T> (t)), doc_(doc), dirty_(false), default_(true), user_supplied_(false)
    {
    }

    template<typename T>
    static tendril::ptr make_tendril()
    {
      holder_base::ptr h(new holder<T>(T()));
      tendril::ptr t(new tendril(h));
      t->dirty_ = false;
      t->default_ = false;
      t->doc_ = "TODO docstring me, please.";
      t->user_supplied_ = false;
      return t;
    }

    /**
     * \brief Copy the tendril by value.
     * @param rhs the tendril to copy from
     */
    tendril(const tendril& rhs);

    /**
     * \brief Copies the tendril by value, so that the two tendrils will point to the same
     *  data.
     * @param rhs
     * @return this
     */
    tendril& operator=(const tendril& rhs);

    /**
     * \brief Copies the value of the given tendril into this one.
     * @param rhs
     */
    void copy_value(const tendril& rhs);

    /**
     * \brief This is an unmangled type name for what ever tendril is
     * holding.
     *
     * @return the unmangled name, e.g. "cv::Mat", or
     * "pcl::PointCloud<pcl::PointXYZ>"
     */
    inline const std::string& type_name() const
    {
      return holder_->type_name();
    }

    /**
     * \brief A doc string for this tendril, "foo is for the input
     * and will be mashed with spam."
     * @return A very descriptive human readable string of whatever
     * the tendril is holding on to.
     */
    inline const std::string& doc() const
    {
      return doc_;
    }

    /**
     * \brief The doc for this tendril is runtime defined, so you may want to update it.
     * @param doc_str A human readable description of the tendril.
     */
    void set_doc(const std::string& doc_str);

    template<typename T>
    void set_default_val(const T& val = T())
    {
      if(!user_supplied_) //user supplied?
      {
        default_ = true;
        holder_.reset(new holder<T>(val));
      }
    }
    /**
     * Given T this will get the type from the tendril, also enforcing type with an exception.
     * @return a const reference to the value of the tendril (no copies)
     */
    template<typename T>
    inline const T& get() const
    {
      return read<T>();
    }

    /**
     * Given T this will get the type from the tendril, also enforcing type with an exception.
     * It is assumed if this is called that the value will be changed, and therefore dirty.
     * If it is not intended to change the tendril, then one should call the read<T> function explicitly.
     * @return a reference to the value of the tendril (no copies)
     */
    template<typename T>
    inline T& get()
    {
      //throws on failure
      enforce_type<T> ();
      mark_dirty(); // likely changed..
      //cast a void pointer to this type.
      return *static_cast<T*> (holder_->get());
    }

    /**\brief Read only access to the tendril.
     * @return a const reference to the value of the tendril (no copies)
     */
    template<typename T>
    inline const T& read() const
    {
      //throws on failure
      enforce_type<T> ();
      //cast a void pointer to this type.
      return *static_cast<const T*> (holder_->get());
    }
    /**
     * \brief runtime check if the tendril is of the given type.
     * @return true if it is the type.
     */
    template<typename T>
    inline bool is_type() const
    {
      return holder_base::check<T>(*holder_);
    }

    /**
     * \brief Test if the given tendril is the same type as this one
     * @param rhs The tendril to test against.
     * @return true if they are the same type.
     */
    inline bool same_type(const tendril& rhs) const
    {
      return type_name() == rhs.type_name();
    }

    inline bool compatible_type(const tendril& rhs) const
    {
      if (same_type(rhs))
        return true;
      return is_type<boost::python::object> () || rhs.is_type<boost::python::object> ();
    }

    inline void enforce_compatible_type(const tendril& rhs) const
    {
      if (!compatible_type(rhs))
        {
          throw std::logic_error(std::string(type_name() + " is not a " + rhs.type_name()).c_str());

        }
    }

    /**
     * \brief runtime check if the tendril is of the given type, this will throw.
     */
    template<typename T>
    inline void enforce_type() const
    {
      if (!is_type<T> ())
        throw std::logic_error(std::string(type_name() + " is not a " + name_of<T> ()).c_str());
    }
    /**
     * \brief Get the boost::python version of the object (by value)
     * @return A copy of the underlying object as a boost python object, will be None type if the conversion fails.
     */
    boost::python::object extract() const;
    /**
     * \brief Set this tendril's value from the python object. This will copy the value
     * @param o a python object holding a type compatible_type with this tendril. Will throw if the types are not compatible_type.
     */
    void set(boost::python::object o);

    bool dirty() const
    {
      return dirty_;
    }

    bool clean() const
    {
      return !dirty_;
    }

    bool user_supplied() const
    {
      return user_supplied_;
    }

    bool has_default() const
    {
      return default_;
    }

    //! A none type for tendril when the tendril is uninitialized.
    struct none
    {
    };

    /**
     * Register a typed callback with the tendril... Will throw on wrong type.
     * @param cb May be called by the notify function, if the tendril is dirty.
     * @return  this
     */
    template<typename T>
    tendril& set_callback(boost::function<void(T)> cb);
    /**
     * \brief Notify the callback, only if this is dirty.
     */
    void notify();

  private:

    // ############################### NVI ####################################
    struct holder_base
    {
      typedef boost::shared_ptr<holder_base> ptr;

      holder_base()
      {
      }
      holder_base& operator=(const holder_base& rhs);
      virtual ~holder_base();
      virtual const std::string& type_name() const = 0;
      virtual void* get() = 0;
      virtual bool is_type(std::type_info const& ti) const = 0;
      virtual void setPython(boost::python::object o) = 0;
      virtual boost::python::object getPython() const = 0;
      virtual void copy_to(holder_base& holder) const = 0;
      virtual ptr clone() const = 0;
      virtual void trigger_callback() = 0;

      template<typename T>
      const T&
      getT() const
      {
        void* tval = const_cast<holder_base*> (this)->get();
        if (tval == NULL)
          throw std::logic_error(type_name() + " is not safe when getting by type");
        return *static_cast<T*> (tval);
      }

      template<typename T>
      T&
      getT()
      {
        void* tval = get();

        if (tval == NULL)
          throw std::logic_error(type_name() + " is not safe when getting by type");

        return *static_cast<T*> (tval);
      }

      //convenience functions for checking types
      template<typename T>
      static bool inline check(holder_base& i);

      template<typename T>
      static inline void checkThrow(holder_base& i) throw (std::logic_error);
    };

    template<typename T>
    struct holder: holder_base
    {
      holder(const T& t);
      const std::string& type_name() const;
      bool is_type(std::type_info const& ti) const;
      void* get();
      void setPython(boost::python::object o);
      boost::python::object getPython() const;
      void copy_to(holder_base& holder) const;
      boost::shared_ptr<holder_base> clone() const;
      void trigger_callback()
      {
        if (cb)
          cb(t);
      }
      T t;
      boost::function<void(T)> cb;
    };
    static holder_base::ptr none_holder_;
    void mark_dirty()
    {
      dirty_ = true;
      user_supplied_ = true;
    }
    void mark_clean()
    {
      dirty_ = false;
    }
    tendril(holder_base::ptr impl);
    boost::shared_ptr<holder_base> holder_;
    std::string doc_;
    bool dirty_,default_, user_supplied_;

  };


  template<typename T>
  bool tendril::holder_base::check(tendril::holder_base& i)
  {
    return i.is_type(typeid(T));
  }

  template<typename T>
  void tendril::holder_base::checkThrow(tendril::holder_base& i) throw (std::logic_error)
  {
    if (!check<T> (i))
      throw std::logic_error(std::string(i.type_name() + " is not a " + name_of<T> ()).c_str());
  }

  template<typename T>
  tendril::holder<T>::holder(const T& t) :
    t(t)
  {
  }

  template<typename T>
  const std::string& tendril::holder<T>::type_name() const
  {
    static const std::string name = name_of<T> ();
    return name;
  }

  template<typename T>
  bool tendril::holder<T>::is_type(std::type_info const& ti) const
  {
    return std::strcmp(typeid(T).name(), ti.name()) == 0;
  }

  template<typename T>
  void*
  tendril::holder<T>::get()
  {
    return &t;
  }

  template<>
  inline void*
  tendril::holder<boost::python::object>::get()
  {
    return NULL; //unsafe to dereference the boost python object.
  }

  template<typename T>
  void tendril::holder<T>::setPython(boost::python::object o)
  {
    boost::python::extract<T> get_T(o);
    if (get_T.check())
      t = get_T();
    else
      throw std::logic_error("Could not convert python object to type : " + type_name());
  }

  template<>
  inline void tendril::holder<boost::python::object>::setPython(boost::python::object o)
  {
    t = o;
  }

  template<typename T>
  boost::python::object tendril::holder<T>::getPython() const
  {
    try
      {
        boost::python::object o(t);
        return o;
      } catch (const boost::python::error_already_set&)
      {
        //silently handle no python wrapping
        PyErr_Clear(); //Need to clear the error or python craps out. Try commenting out and running the doc tests.
      }
    return boost::python::object();
  }

  template<>
  inline boost::python::object tendril::holder<boost::python::object>::getPython() const
  {
    return t;
  }

  template<typename T>
  void tendril::holder<T>::copy_to(holder_base& holder) const
  {
    void* tval = holder.get();
    if (tval != NULL)
      *static_cast<T*> (tval) = t;
    else
      holder.setPython(getPython());
  }

  template<>
  inline void tendril::holder<boost::python::object>::copy_to(holder_base& holder) const
  {
    holder.setPython(t);
  }

  template<typename T>
  tendril::holder_base::ptr tendril::holder<T>::clone() const
  {
    tendril::holder_base::ptr p(new holder<T> (t));
    return p;
  }

  template<typename T>
  tendril& tendril::set_callback(boost::function<void(T)> cb)
  {
    typedef holder<T> holder_t;
    enforce_type<T> ();
    holder_base* hb = holder_.get();
    holder_t* ht = dynamic_cast<holder_t*> (hb);
    ht->cb = cb;
    return *this;
  }

}

template<typename T>
ecto::tendril& operator<<(ecto::tendril& t, boost::function<void(T)> cb)
{
  t.set_callback(cb);
  return t;
}
