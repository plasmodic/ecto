#include <ecto/tendril.hpp>

namespace ecto
{
  tendril::tendril() :
    //impl_ is never not initialized
        impl_(new impl<none> (none())),connected_(false)
  {
  }

  tendril::tendril(impl_base::ptr impl) :
    impl_(impl),connected_(false)
  {
  }

  std::string tendril::type_name() const
  {
    return impl_->type_name();
  }

  std::string tendril::doc() const
  {
    return impl_->doc;
  }

  void tendril::setDoc(const std::string& doc_str)
  {
    impl_->doc = doc_str;
  }
  void tendril::connect(tendril& rhs)
  {

    if (type_name() != rhs.type_name())
    {
      if (rhs.is_type<boost::python::object> () && impl_->steal(rhs))
      {
      }
      else if (is_type<boost::python::object> () && rhs.impl_->steal(*this))
      {
      }
      else
      {
        throw std::runtime_error("bad connect! input(" + impl_->type_name() + ") != output(" + rhs.type_name() + ")");
      }
    }
    if( !connected_)
    {
      impl_ = rhs.impl_;
    }
    else
    {
      throw std::runtime_error("both of these are already connected.");
    }
    connected_ = true;
  }

  tendril::impl_base::~impl_base()
  {
  }

  boost::python::object tendril::extract() const
  {
    return impl_->getPython();
  }

  void tendril::set(boost::python::object o)
  {
    if (is_type<none> ())
    {
      impl_.reset(new impl<boost::python::object> (o));
    }
    else
    {
      impl_->setPython(o);
    }
  }
}
