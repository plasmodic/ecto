#include <ecto/tendril.hpp>

#define pre_check(x) \
do{ \
if(!(x).impl_) \
    throw std::logic_error("This connection is uninitialized!"); \
}while(false)
namespace ecto
{
  connection::connection() :
    dirty_(true)
  {
  }
  connection::connection(impl_base::ptr impl) :
    impl_(impl), dirty_(true)
  {
  }
  std::string connection::type_name() const
  {
    pre_check(*this);
    return impl_->type_name();
  }
  std::string connection::value() const
  {
    pre_check(*this);
    return impl_->value();
  }
  std::string connection::name() const
  {
    pre_check(*this);
    return impl_->name;
  }
  std::string connection::doc() const
  {
    pre_check(*this);
    return impl_->doc;
  }

  void connection::connect(connection& rhs)
  {
    pre_check(*this);
    pre_check(rhs);
    if(impl_->type_name() != rhs.impl_->type_name())
      throw std::runtime_error("bad connect! input(" + impl_->type_name() + ") != output(" + rhs.type_name() +")");
    impl_ = rhs.impl_;
  }

  connection::impl_base::~impl_base()
  {
  }
}
