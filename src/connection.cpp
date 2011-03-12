#include "ecto/connection.hpp"
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
    return impl_->type_name();
  }
  std::string connection::value() const
  {
    return impl_->value();
  }
  std::string connection::name() const
  {
    return impl_->name;
  }
  std::string connection::doc() const
  {
    return impl_->doc;
  }

  void connection::connect(connection& rhs)
  {
    if(impl_->type_info() != rhs.impl_->type_info())
      throw std::runtime_error("bad connect! input(" + impl_->type_name() + ") != output(" + rhs.type_name() +")");
    impl_ = rhs.impl_;
  }

  connection::impl_base::~impl_base()
  {
  }
}
