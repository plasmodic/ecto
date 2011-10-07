#include <ecto/tendril.hpp>
#include <boost/python.hpp>
namespace ecto
{
  using namespace except;

  tendril::tendril()
    : doc_()
    , flags_()
    , converter(&ConverterImpl<none>::instance)
  {
    set_holder<none>(none());
  }

  tendril::tendril(const tendril& rhs) 
    : holder_(rhs.holder_)
    , type_ID_(rhs.type_ID_)
    , doc_(rhs.doc_)
    , flags_(rhs.flags_)
    , converter(rhs.converter)
  { }

  tendril& tendril::operator=(const tendril& rhs)
  {
    if (this == &rhs)
      return *this;
    copy_holder(rhs);
    doc_ = rhs.doc_;
    flags_ = rhs.flags_;
    converter = rhs.converter;
    return *this;
  }

  tendril::~tendril(){ }



  ecto::tendril& tendril::operator<<(const tendril& rhs)
  {
    if (this == &rhs)
      return *this;
    if (is_type<none>() || same_type(rhs))
    {
      copy_holder(rhs);
    }
    else
    {
      enforce_compatible_type(rhs);
      if (rhs.is_type<none>())
      {
        BOOST_THROW_EXCEPTION(ecto::except::ValueNone());
      }
      else if (rhs.is_type<boost::python::object>())
      {
        *this << rhs.get<boost::python::object>();
      }
      else if (is_type<boost::python::object>())
      {
        (*rhs.converter)(*boost::unsafe_any_cast<boost::python::object>(&holder_), rhs);
      }
    }
    user_supplied(true);
    return *this;
  }


  void tendril::set_doc(const std::string& doc_str)
  {
    doc_ = doc_str;
  }

  void tendril::notify()
  {
    if (dirty())
    {
      jobs_(*this);
    }
    dirty(false);
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
    return flags_[REQUIRED];
  }

  void
  tendril::required(bool b)
  {
    flags_[REQUIRED] = b;
  }

  bool
  tendril::user_supplied() const
  {
    return flags_[USER_SUPPLIED];
  }

  void
  tendril::user_supplied(bool b)
  {
    flags_[USER_SUPPLIED] = b;
  }

  bool
  tendril::has_default() const
  {
    return flags_[DEFAULT_VALUE];
  }

  bool
  tendril::dirty() const
  {
    return flags_[DIRTY];
  }

  void
  tendril::dirty(bool dirty)
  {
    flags_[DIRTY] = dirty;
  }

  bool
  tendril::clean() const
  {
    return !flags_[DIRTY];
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
      BOOST_THROW_EXCEPTION(except::TypeMismatch() << from_typename(type_name())
                            << to_typename(rhs.type_name()));
        ;
    }
  }

  void tendril::copy_holder(const tendril& rhs)
  {
    holder_ = rhs.holder_;
    type_ID_ = rhs.type_ID_;
    converter = rhs.converter;
  }

  void tendril::operator<<(const boost::python::object& obj)
  {
    if (is_type<boost::python::object>())
      {
        holder_ = obj;
      }
    else if (is_type<none>())
      {
        set_holder(obj);
      }
    else
      (*converter)(*this, obj);
  }

  void
  operator>>(const tendril_ptr& rhs, boost::python::object& obj)
  {
    if (!rhs)
      BOOST_THROW_EXCEPTION(except::NullTendril() 
                            << except::from_typename("(null)")
                            << except::to_typename("(python object)"));
    *rhs >> obj;
  }

  // e.g.  ISO C++ says that these are ambiguous, even...
  void
  operator>>(const tendril_cptr& rhs, boost::python::object& obj)
  {
    if (!rhs)
      BOOST_THROW_EXCEPTION(except::NullTendril() 
                            << except::from_typename("(null)")
                            << except::to_typename("(python object)"));
    *rhs >> obj;
  }
  void
  operator<<(const tendril_ptr& lhs, const tendril_ptr& rhs)
  {
    if (!lhs)
      BOOST_THROW_EXCEPTION(except::NullTendril() 
                            << except::to_typename("(null)")
                            << except::from_typename(rhs ? rhs->type_name() : "(null)"));
    if (!rhs)
      BOOST_THROW_EXCEPTION(except::NullTendril()
                            << except::to_typename(lhs->type_name())
                            << except::from_typename("(null)"));
    *lhs << *rhs;
  }

  void
  operator<<(const tendril_ptr& lhs, const tendril_cptr& rhs)
  {
    if (!lhs)
      BOOST_THROW_EXCEPTION(except::NullTendril() 
                            << except::to_typename("(null)")
                            << except::from_typename(rhs->type_name()));
    if (!rhs)
      BOOST_THROW_EXCEPTION(except::NullTendril() 
                            << except::to_typename(lhs->type_name())
                            << except::from_typename("(null)"));
    *lhs << *rhs;
  }



}
