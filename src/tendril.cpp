#include <ecto/tendril.hpp>

#define pre_check(x) \
do{ \
if(!(x).impl_) \
    throw std::logic_error("This connection is uninitialized!"); \
}while(false)
namespace ecto
{

  std::ostream& operator<<(std::ostream&out, const tendril::none& rhs)
  {
    return out << "none";
  }
  tendril::tendril() :
    impl_(new impl<none>(none())), dirty_(true)
  {
  }
  tendril::tendril(impl_base::ptr impl) :
    impl_(impl), dirty_(true)
  {
  }
  std::string tendril::type_name() const
  {
    pre_check(*this);
    return impl_->type_name();
  }
  std::string tendril::value() const
  {
    pre_check(*this);
    return impl_->value();
  }
  std::string tendril::name() const
  {
    pre_check(*this);
    return impl_->name;
  }
  std::string tendril::doc() const
  {
    pre_check(*this);
    return impl_->doc;
  }

  void tendril::connect(tendril& rhs)
  {
    pre_check(*this);
    pre_check(rhs);
    if (impl_->type_name() != rhs.impl_->type_name())
      throw std::runtime_error("bad connect! input(" + impl_->type_name() + ") != output(" + rhs.type_name() + ")");
    impl_ = rhs.impl_;
  }

  tendril::impl_base::~impl_base()
  {
  }
}
