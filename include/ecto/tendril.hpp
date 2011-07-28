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
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/any.hpp>

#include <ecto/util.hpp> //name_of
#include <ecto/except.hpp>

#include <vector>
#include <string>

namespace ecto
{
  /**
   * \brief A tendril is the slender, winding organ of the
   * ecto::cell that gives it its awesome type erasure and uber
   * flexibility.
   *
   * Each tendril is a type erasing holder for any instance of any type,
   * and allows introspection including its value, type, and doc string.
   *
   * The tendril operates as a value holder, so treat it as such. If you would like to pass it around without copies,
   * construct a pointer to tendril, perhaps with the make_tendril<T>() function.
   *
   * Items held by the tendril must be copy constructible and copiable.
   */
  class ECTO_EXPORT tendril
  {
  public:
    typedef boost::shared_ptr<tendril> ptr;
    typedef boost::shared_ptr<const tendril> const_ptr;
    typedef boost::function<void(tendril&)> TendrilJob;
    /**
     * \brief Creates a tendril that is initialized with the
     * tendril::none type. This should be fairly cheap.
     */
    tendril();
    /**
     * \brief Will deallocate the value held.
     */
    ~tendril();

    tendril(const tendril& rhs);
    tendril& operator=(const tendril& rhs);

    /**
     * \brief A a convenience constructor for creating a tendril
     * that holds the given type.
     * @tparam T The type to hide in this tendril
     * @param t default value for t
     * @param doc a documentation string
     */
    template<typename T>
    tendril(const T& t, const std::string& doc);


    /**
     * \brief This is an unmangled type name for what ever tendril is
     * holding.
     *
     * @return the unmangled name, e.g. "cv::Mat", or
     * "pcl::PointCloud<pcl::PointXYZ>"
     */
    std::string
    type_name() const;

    /**
     * \brief A doc string for this tendril, "foo is for the input
     * and will be mashed with spam."
     * @return A very descriptive human readable string of whatever
     * the tendril is holding on to.
     */
    std::string
    doc() const;

    /**
     * \brief The doc for this tendril is runtime defined, so you may want to update it.
     * @param doc_str A human readable description of the tendril.
     */
    void
    set_doc(const std::string& doc_str);

    /**
     * \brief This sets the default value of the tendril. This is a
     * @param val
     */
    template<typename T>
    void
    set_default_val(const T& val = T())
    {
      enforce_type<T>();
      if (!user_supplied_) //user supplied?
      {
        default_ = true;
        set_holder<T>(val);
      }
    }

    void required(bool b);

    bool required() const;

    /**
     * Given T this will get the type from the tendril, also enforcing type with an exception.
     * @return a const reference to the value of the tendril (no copies)
     */
    template<typename T>
    inline const T&
    get() const
    {
      enforce_type<T>();
      return *boost::unsafe_any_cast<const T>(&holder_);
    }

    template<typename T>
    inline T&
    get() 
    {
      enforce_type<T>();
      return *boost::unsafe_any_cast<T>(&holder_);
    }

    template <typename T> 
    void 
    operator<<(const T& val)
    {
      if(is_type<none>()) //handle none case.
      {
        set_holder<T>(val);
      }else
      {
        //throws on failure
        enforce_type<T>();
        //cast a void pointer to this type.
        *boost::unsafe_any_cast<T>(&holder_) = val;
      }
      mark_dirty(); //definitely changed
    }

    void operator<<(const boost::python::object& obj)
    {
      if (is_type<boost::python::object>())
        holder_ = obj;
      else if (is_type<none>())
        set_holder(obj);
      else
        (*converter)(*this, obj);
    }

    ecto::tendril& operator<<(const ecto::tendril& rhs);

    /**
     * \brief runtime check if the tendril is of the given type.
     * @return true if it is the type.
     */
    template<typename T>
    bool
    is_type() const
    {
      const char* ourname = name_of<T>().c_str();
      return ourname == type_ID_;
    }

    /**
     * \brief Test if the given tendril is the same type as this one
     * @param rhs The tendril to test against.
     * @return true if they are the same type.
     */
    bool
    same_type(const tendril& rhs) const;

    bool
    compatible_type(const tendril& rhs) const;

    void
    enforce_compatible_type(const tendril& rhs) const;

    /**
     * \brief runtime check if the tendril is of the given type, this will throw.
     */
    template<typename T>
    inline void
    enforce_type() const
    {
      if (!is_type<T>())
        throw except::TypeMismatch(type_name() + " is not a " + name_of<T>());
    }

    //! The value that this tendril holds was supplied by the user at some point.
    bool
    user_supplied() const;

    void user_supplied(bool v);

    //! The tendril was initialized with default value.
    bool
    has_default() const;

    //! A none type for tendril when the tendril is uninitialized.
    struct none { 
      none& operator=(const none&) { return *this; }
      const none& operator=(const none&) const { return *this; } // funny const assignment operator
      friend bool operator==(const none&, const none&) { return true; }
      friend std::ostream& operator<<(std::ostream& os, const none&) 
      { 
        os << "ecto::tendril::none"; return os;
      }
    };


    void enqueue_oneshot(TendrilJob job);
    void enqueue_persistent(TendrilJob job);

    void exec_oneshots();
    void exec_persistent();

    template<typename T>
    struct Caller
    {
      typedef typename boost::function<void(T)> CbT;
      Caller(CbT cb)
        : cb(cb)
      { }

      void
      operator()(tendril& t)
      {
        cb(t.get<T>());
      }
      CbT cb;
    };

    /**
     * Register a typed callback with the tendril... Will throw on wrong type.
     * @param cb May be called by the notify function, if the tendril is dirty.
     * @param oneshot do only once
     * @return  this 
     */
    template<typename T>
    tendril&
    set_callback(typename boost::function<void(T)> cb, bool oneshot = false)
    {
      typedef Caller<T> CallerT;
      enforce_type<T>();
      if(oneshot)
        enqueue_oneshot(CallerT(cb));
      else
        enqueue_persistent(CallerT(cb));
      return *this;
    }

    //! Notify the callback, only if this is dirty.
    void
    notify();

    //! The tendril has likely been modified since the last time that notify has beend called.
    bool
    dirty() const;

    //! The tendril has notified its callback if one was registered since it was changed.
    bool
    clean() const;

  private:

    template<typename T>
    inline const T& unsafe_get() const
    {
      return *boost::unsafe_any_cast<const T>(&holder_);
    }

    template<typename T>
    inline T& unsafe_get()
    {
      return *boost::unsafe_any_cast<const T>(&holder_);
    }

    struct Converter
    {
      virtual void operator()(tendril& t, const boost::python::object& o) const = 0;
      virtual void operator()(boost::python::object& o, const tendril& t) const = 0;
    };
    
    template <typename T,  typename _=void>
    struct ConverterImpl : Converter
    {
      static ConverterImpl<T, _> instance;

      void
      operator()(tendril& t, const boost::python::object& obj) const
      {
        boost::python::extract<T> get_T(obj);
        if (get_T.check())
          t << get_T();
        else
          throw ecto::except::TypeMismatch("Could not convert python object to type : " + t.type_name());
      }

      void
      operator()(boost::python::object& o, const tendril& t) const
      {
        const T& v = t.unsafe_get<T>();
        boost::python::object obj(v);
        o = obj;
      }
    };

    template <typename _>
    struct ConverterImpl<none, _> : Converter
    {
      static ConverterImpl<none, _> instance;

      void
      operator()(tendril& t, const boost::python::object& obj) const
      {
        t << obj;
      }

      void
      operator()(boost::python::object& o, const tendril& t) const
      {
        o = boost::python::object();
      }
    };

    template <typename T>
    void set_holder(const T& t = T())
    {
      holder_ = t;
      type_ID_ = name_of<T>().c_str();
      converter = &ConverterImpl<T>::instance;
    }

    void copy_holder(const tendril& rhs);
    void mark_dirty();
    void mark_clean();

    boost::any holder_;
    const char* type_ID_;
    std::string doc_;
    bool dirty_, default_, user_supplied_, required_;
    std::vector<TendrilJob> jobs_onetime_,jobs_persistent_;
    boost::mutex mtx_;
    Converter* converter;

  public:

    template <typename T>
    void operator>>(T& val) const
    {
      enforce_type<T>();
      val = *boost::unsafe_any_cast<T>(&holder_);
    }

    void operator>>(boost::python::object& obj) const
    {
      (*converter)(obj, *this); 
    }

    void operator>>(ecto::tendril::ptr rhs) const
    {
      *rhs << *this;
    }

    void operator>>(ecto::tendril& rhs) const
    {
      rhs << *this;
    }

    template<typename T>
      friend void
      operator>>(ecto::tendril::ptr rhs, T& val)
    {
      if (!rhs)
        throw std::runtime_error("Attempt to convert from tendril which is null");
      *rhs >> val;
    }

    template<typename T>
    friend void
    operator>>(ecto::tendril::const_ptr rhs, T& val)
    {
      if (!rhs)
        throw std::runtime_error("Attempt to convert from tendril which is null");
      *rhs >> val;
    }

    friend void
    operator>>(ecto::tendril::ptr rhs, boost::python::object& obj)
    {
      if (!rhs)
        throw std::runtime_error("Attempt to convert from tendril which is null");
      *rhs >> obj;
    }

    friend void
    operator>>(ecto::tendril::const_ptr rhs, boost::python::object& obj)
    {
      if (!rhs)
        throw std::runtime_error("Attempt to convert from tendril which is null");
      *rhs >> obj;
    }

    template<typename T>
    friend void
    operator<<(ecto::tendril::ptr lhs, const T& rhs)
    {
      if (!lhs)
        throw std::runtime_error("Attempt to convert into tendril which is null");
      *lhs << rhs;
    }


    friend void
    operator<<(ecto::tendril::ptr lhs, const ecto::tendril& rhs)
    {
      if (!lhs)
        throw std::runtime_error("Attempt to convert into tendril which is null");
      *lhs << rhs;
    }

    friend void
    operator<<(ecto::tendril::ptr lhs, const ecto::tendril::ptr& rhs)
    {
      *lhs << *rhs;
    }

    template<typename T>
    friend
    tendril::ptr make_tendril()
    {
      tendril::ptr t(new tendril());
      t->set_holder<T>();
      return t;
    }
  };

  template <typename T, typename _> tendril::ConverterImpl<T,_> tendril::ConverterImpl<T,_>::instance;

  template <typename _> tendril::ConverterImpl<tendril::none,_> tendril::ConverterImpl<tendril::none,_>::instance;

  template<typename T>
  tendril::tendril(const T& t, const std::string& doc)
    : dirty_(false)
    , default_(true)
    , user_supplied_(false)
    , converter(&ConverterImpl<T>::instance)
  {
    set_holder<T>(t);
    set_doc(doc);
  }

}

