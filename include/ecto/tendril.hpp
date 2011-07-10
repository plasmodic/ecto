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
#include <ecto/tags.hpp>

#include <string>
#include <cstring>

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
  class tendril
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

    template<typename T>
    static tendril::ptr
    make_tendril()
    {
      tendril::ptr t(new tendril());
      t->holder_ = T();
      t->pycopy_to_ = ToPython<T>::Copier.get();
      t->pycopy_from_ = FromPython<T>::Copier.get();
      return t;
    }

    /**
     * \brief Copies the value of the given tendril into this one.
     * @param rhs
     */
    void
    copy_value(const tendril& rhs);

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
        holder_ = val;
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
      return read<T>();
    }

    /**
     * Given T this will get the type from the tendril, also enforcing type with an exception.
     * It is assumed if this is called that the value will be changed, and therefore dirty.
     * If it is not intended to change the tendril, then one should call the read<T> function explicitly.
     * @return a reference to the value of the tendril (no copies)
     */
    template<typename T>
    inline T&
    get()
    {
      //throws on failure
      enforce_type<T>();
      mark_dirty(); // likely changed..
      //cast a void pointer to this type.
      return boost::any_cast<T&>(holder_);
    }

    /**\brief Read only access to the tendril.
     * @return a const reference to the value of the tendril (no copies)
     */
    template<typename T>
    inline const T&
    read() const
    {
      //throws on failure
      enforce_type<T>();
      //cast a void pointer to this type.
      return  boost::any_cast<const T&>(holder_);
    }

    template<typename T>
    void
    sample(T& val) const
    {
      //throws on failure
      enforce_type<T>();
      //cast a void pointer to this type.
      val = boost::any_cast<const T&>(holder_);
    }

    template<typename T>
    void
    set(const T& val)
    {
      //throws on failure
      enforce_type<T>();
      //cast a void pointer to this type.
      boost::any_cast<T&>(holder_) = val;
      mark_dirty(); //definitely changed
    }

    /**
     * \brief runtime check if the tendril is of the given type.
     * @return true if it is the type.
     */
    template<typename T>
    bool
    is_type() const
    {
      return 0 == std::strcmp(holder_.type().name(), typeid(T).name());
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

    //! The tendril was initialized with default value.
    bool
    has_default() const;

    //! A none type for tendril when the tendril is uninitialized.
    struct none
    {
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
          :
            cb(cb)
      {
      }
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

    tags::tags& tags();
    tendril& tag(const tags::tags_base& c);
    tags::ptr get_tag(const char* key) const;

    template <typename T>
    const T& tagged(const tags::tag_<T>& _c) const
    {
      return tags_.tagged(_c);
    }

    void sample(boost::python::object&) const;
    void set(const boost::python::object&);
  private:

    struct PyCopier_base
    {
      virtual
      void operator()(tendril& t, boost::python::object& obj) = 0;
    };

    template<typename T>
    struct ToPython: PyCopier_base
    {
      static boost::scoped_ptr<PyCopier_base> Copier;
      void
      operator()(tendril& t, boost::python::object& obj)
      {
        const T& v = t.read<T>();
        boost::python::object o(v);
        obj = o;
      }
    };

    template<typename T>
    struct FromPython: PyCopier_base
    {
      static boost::scoped_ptr<PyCopier_base> Copier;
      void
      operator()(tendril& t, boost::python::object& obj)
      {
        boost::python::extract<T> get_T(obj);
        if (get_T.check())
          t.get<T>() = get_T();
        else
          throw ecto::except::TypeMismatch("Could not convert python object to type : " + t.type_name());
      }
    };

    void
    mark_dirty();
    void
    mark_clean();
    boost::any holder_;
    bool dirty_, default_, user_supplied_;
    std::vector<TendrilJob> jobs_onetime_,jobs_persistent_;
    boost::mutex mtx_;
    tags::tags tags_;
    PyCopier_base* pycopy_to_,* pycopy_from_;
  };

  template<typename T>
  tendril::tendril(const T& t, const std::string& doc)
      :
        holder_(t),
        dirty_(false),
        default_(true),
        user_supplied_(false)
  {
    set_doc(doc);
    pycopy_to_ = ToPython<T>::Copier.get();
    pycopy_from_ = FromPython<T>::Copier.get();
  }

  template<typename T>
  boost::scoped_ptr<tendril::PyCopier_base> tendril::ToPython<T>::Copier(new ToPython<T>());
  template<typename T>
  boost::scoped_ptr<tendril::PyCopier_base> tendril::FromPython<T>::Copier(new FromPython<T>());
}

template<typename T>
void
operator<<(ecto::tendril& rhs,const T& val)
{
  rhs.get<T>() = val;
}

template<typename T>
void
operator<<(const ecto::tendril::ptr& rhs,const T& val)
{
  if(!rhs) throw std::runtime_error("Your tendril be null!");
  rhs->get<T>() = val;
}

template<typename T>
void
operator>>(const ecto::tendril& rhs,T& val)
{
  val = rhs.read<T>();
}

template<typename T>
void
operator>>(const ecto::tendril::ptr& rhs,T& val)
{
  if(!rhs) throw std::runtime_error("Your tendril be null!");
  val = rhs->read<T>();
}
