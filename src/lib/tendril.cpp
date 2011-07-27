#include <ecto/tendril.hpp>
#include <boost/python.hpp>
namespace ecto
{

  namespace
  {
    bool isBoostPython(const tendril& t)
    {
      return t.is_type<boost::python::object>();
    }
  }

  tendril::tendril()
    : doc_()
    , dirty_(false)
    , default_(false)
    , user_supplied_(false)
    , required_(false)
    , frompy_convert(&FromPython<none>::copier)
    , topy_convert(&ToPython<none>::copier)
  {
    set_holder<none>(none());
  }

  tendril::~tendril(){ }

  tendril::tendril(const tendril& rhs) 
     : doc_(rhs.doc_)
     , dirty_(false)
     , default_(rhs.default_)
     , user_supplied_(rhs.user_supplied_)
     , required_(rhs.required_)
     , frompy_convert(rhs.frompy_convert)
     , topy_convert(rhs.topy_convert)
  {
    copy_holder(rhs);
  }

  tendril& tendril::operator=(const tendril& rhs)
  {
    if (this == &rhs)
      return *this;
    copy_holder(rhs);
    doc_ = rhs.doc_;
    dirty_ = rhs.dirty_;
    default_ = rhs.default_;
    required_ = rhs.required_;
    frompy_convert = rhs.frompy_convert;
    topy_convert = rhs.topy_convert;
    return *this;
  }

  void tendril::copy_value(const tendril& rhs)
  {
    if (this == &rhs)
      return;
    if (is_type<none>() || same_type(rhs))
    {
      copy_holder(rhs);
    }
    else
    {
      enforce_compatible_type(rhs);
      if (rhs.is_type<none>())
      {
        //throw ecto::except::ValueNone("You may not copy the value of a tendril that holds a tendril::none.");
      }
      else if (rhs.is_type<boost::python::object>())
      {
        *this << rhs.get<boost::python::object>();
      }
      else if (is_type<boost::python::object>())
      {
        //*this << rhs;
        //rhs.sample(get<boost::python::object>());
        (*rhs.topy_convert)(*boost::unsafe_any_cast<boost::python::object>(&holder_), rhs);
      }
    }
    mark_dirty();
  }


  void tendril::set_doc(const std::string& doc_str)
  {
    doc_ = doc_str;
  }

  void tendril::enqueue_oneshot(TendrilJob job)
  {
    boost::mutex::scoped_lock lock(mtx_);
    jobs_onetime_.push_back(job);
  }
  void tendril::enqueue_persistent(TendrilJob job)
  {
    boost::mutex::scoped_lock lock(mtx_);
    jobs_persistent_.push_back(job);
  }

  struct exec
  {
    exec(tendril&t):t(t){}
    void operator()(tendril::TendrilJob& job)
    {
      job(t);
    }
    tendril& t;
  };

  template<typename JobQue>
  void
  execute(tendril& t, JobQue& q)
  {
    std::for_each(q.begin(), q.end(), exec(t));
  }

  void tendril::exec_oneshots()
  {
    boost::mutex::scoped_lock lock(mtx_);
    execute(*this,jobs_onetime_);
    jobs_onetime_.clear();
  }

  void tendril::exec_persistent()
  {
    boost::mutex::scoped_lock lock(mtx_);
    execute(*this,jobs_persistent_);
  }

  void tendril::notify()
  {
    exec_oneshots();
    if (dirty())
    {
      exec_persistent();
    }
    mark_clean();
  }

  std::string
  tendril::doc() const
  {
    return doc_;
  }

  std::string
  tendril::type_name() const
  {
   return type_ID_;
  }

  bool
  tendril::required() const
  {
    return required_;
  }

  void
  tendril::required(bool b)
  {
    required_ = b;
  }

  bool
  tendril::user_supplied() const
  {
    return user_supplied_;
  }

  bool
  tendril::has_default() const
  {
    return default_;
  }

  bool
  tendril::dirty() const
  {
    return dirty_;
  }

  //! The tendril has notified its callback if one was registered since it was changed.
  bool
  tendril::clean() const
  {
    return !dirty_;
  }

  bool
  tendril::same_type(const tendril& rhs) const
  {
    return rhs.type_ID_ == type_ID_;
  }

  bool
  tendril::compatible_type(const tendril& rhs) const
  {
    if (same_type(rhs))
      return true;
    return is_type<none>() || rhs.is_type<none>() || is_type<boost::python::object>()
           || rhs.is_type<boost::python::object>();
  }

  void
  tendril::enforce_compatible_type(const tendril& rhs) const
  {
    if (!compatible_type(rhs))
    {
      throw except::TypeMismatch(type_name() + " is not a " + rhs.type_name());
    }
  }

  void
  tendril::mark_dirty()
  {
    dirty_ = true;
    user_supplied_ = true;
  }
  void
  tendril::mark_clean()
  {
    dirty_ = false;
  }

  void tendril::copy_holder(const tendril& rhs)
  {
    holder_ = rhs.holder_;
    type_ID_ = rhs.type_ID_;
  }

}
