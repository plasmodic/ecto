#include <ecto/tendril.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
namespace ecto
{
tendril::tendril() :
  impl_(new impl<none> (none(), this)), connected_(false)
{
  //impl_ is never not initialized
}

tendril::~tendril()
{
  //SHOW();
  impl_->owners.erase(this);
}

tendril::tendril(const tendril& rhs) :
  impl_(rhs.impl_), connected_(rhs.connected_)
{
  impl_->owners.insert(this);
}
tendril& tendril::operator=(const tendril& rhs)
{
  if (this == &rhs)
    return *this;

  impl_->owners.erase(this);
  impl_ = rhs.impl_;
  impl_->owners.insert(this);
  connected_ = rhs.connected_;
  return *this;
}

tendril::tendril(impl_base::ptr impl) :
  impl_(impl), connected_(false)
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
namespace
{
bool isBoostPython(const tendril& t)
{
  return t.is_type<boost::python::object> ();
}
}
void tendril::connect(tendril& rhs)
{
  if (connected_)
  {
    //TODO should this throw on reconnect?
    throw std::runtime_error("Already connected. This is considered an error.");
    //disconnect();
  }
  boost::shared_ptr<impl_base> impl;
  //check if the types aren't the same
  if (!same_type(rhs))
  {
    //FIXME this sucks for now...
    //if one is a boost::python::object then we may
    //override its impl_
    if (isBoostPython(*this))
    {
      impl = rhs.impl_;
      impl->owners.insert(impl_->owners.begin(), impl_->owners.end());
    }
    else if (isBoostPython(rhs))
    {
      impl = impl_;
      impl->owners.insert(rhs.impl_->owners.begin(), rhs.impl_->owners.end());
    }
    else
      throw std::runtime_error(
          "bad connect! input(" + impl_->type_name() + ") != output("
              + rhs.type_name() + ")");
  }
  else
  {
    impl = rhs.impl_;
    impl->owners.insert(impl_->owners.begin(), impl_->owners.end());
  }

  foreach(tendril* x, impl->owners)
        {
          x->impl_ = impl;
        }
  //set out connected state to true
  connected_ = true;
}

void tendril::disconnect()
{
  //erase all owners
  impl_->owners.erase(this);
  //reset this impl;
  impl_ = impl_->make(this);
  //set the connected_ flag to false
  connected_ = false;
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
    impl_.reset(new impl<boost::python::object> (o, this));
  }
  else
  {
    impl_->setPython(o);
  }
}
}
