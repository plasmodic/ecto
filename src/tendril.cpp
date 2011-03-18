#include <ecto/tendril.hpp>

namespace ecto
{

  std::ostream& operator<<(std::ostream&out, const tendril::none& rhs)
  {
    return out << "none";
  }
  tendril::tendril() :
      //impl_ is never not initialized
    impl_(new impl<none> (none())), dirty_(true)
  {
  }
  tendril::tendril(impl_base::ptr impl) :
    impl_(impl), dirty_(true)
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

  void tendril::connect(tendril& rhs)
  {
    if (impl_->type_name() != rhs.impl_->type_name())
      throw std::runtime_error("bad connect! input(" + impl_->type_name() + ") != output(" + rhs.type_name() + ")");
    impl_ = rhs.impl_;
  }

  tendril::impl_base::~impl_base()
  {
  }

  boost::python::object tendril::extractFromPython()
  {
    return impl_->getPython();
  }
  void tendril::setFromPython(boost::python::object o)
  {
    impl_->setPython(o);
  }
}
