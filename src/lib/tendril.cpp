#include <ecto/tendril.hpp>

namespace ecto
{
  namespace
  {
    bool isBoostPython(const tendril& t)
    {
      return t.is_type<boost::python::object>();
    }
  }

  tendril::tendril() :
      holder_(new holder<none>(none())), dirty_(false), default_(false), user_supplied_(false)
  {
    //impl_ is never not initialized
  }

  tendril::~tendril()
  {
  }

  tendril::tendril(const tendril& rhs) :
      holder_(rhs.holder_), doc_(rhs.doc_), dirty_(false), default_(
          rhs.default_), user_supplied_(rhs.user_supplied_)
  {
  }

  tendril& tendril::operator=(const tendril& rhs)
  {
    if (this == &rhs)
      return *this;
    holder_ = rhs.holder_;
    doc_ = rhs.doc_;
    dirty_ = rhs.dirty_;
    default_ = rhs.default_;
    return *this;
  }

  void tendril::copy_value(const tendril& rhs)
  {
    if (this == &rhs)
      return;
    if (is_type<none>())
    {
      holder_ = rhs.holder_->clone();
    }
    else if (compatible_type(rhs))
    {
      *holder_ = *rhs.holder_;
    }
    mark_dirty();
  }

  tendril::tendril(holder_base::ptr impl) :
      holder_(impl), dirty_(false), default_(false), user_supplied_(false)
  {
  }

  void tendril::set_doc(const std::string& doc_str)
  {
    doc_ = doc_str;
  }

  tendril::holder_base& tendril::holder_base::operator=(
      const tendril::holder_base& rhs)
  {
    if (this == &rhs)
      return *this;
    rhs.copy_to(*this);
    return *this;
  }

  tendril::holder_base::~holder_base()
  {
  }

  boost::python::object tendril::extract() const
  {
    return holder_->getPython();
  }

  void tendril::set(boost::python::object o)
  {
    if (is_type<none>())
    {
      holder_.reset(new holder<boost::python::object>(o));
    }
    else
    {
      holder_->setPython(o);
    }
    mark_dirty();
  }
  void tendril::trigger_callback()
  {
    if (dirty())
    {
      holder_->trigger_callback();
    }
    mark_clean();
  }
}
