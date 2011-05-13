#include <ecto/tendril.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
namespace ecto
{
tendril::tendril() :
  holder_(new holder<none> (none(), this)), connected_(false)
{
  //impl_ is never not initialized
}

tendril::~tendril()
{
  //SHOW();
  holder_->release(this);
}

tendril::tendril(const tendril& rhs) :
  holder_(rhs.holder_), connected_(rhs.connected_)
{
  holder_->claim(this);
}
tendril& tendril::operator=(const tendril& rhs)
{
  if (this == &rhs)
    return *this;

  holder_->release(this);
  holder_ = rhs.holder_;
  holder_->claim(this);
  connected_ = rhs.connected_;
  return *this;
}

tendril::tendril(holder_base::ptr impl) :
  holder_(impl), connected_(false)
{
}

std::string tendril::type_name() const
{
  return holder_->type_name();
}

std::string tendril::doc() const
{
  return holder_->doc;
}

void tendril::setDoc(const std::string& doc_str)
{
  holder_->doc = doc_str;
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
  boost::shared_ptr<holder_base> impl;
  //check if the types aren't the same
  if (!same_type(rhs))
  {
    //FIXME this sucks for now...
    //if one is a boost::python::object then we may
    //override its impl_
    if (isBoostPython(*this))
    {
      impl = rhs.holder_;
      impl->owners.insert(holder_->owners.begin(), holder_->owners.end());
    }
    else if (isBoostPython(rhs))
    {
      impl = holder_;
      impl->owners.insert(rhs.holder_->owners.begin(), rhs.holder_->owners.end());
    }
    else
      throw std::runtime_error(
          "bad connect! input(" + holder_->type_name() + ") != output("
              + rhs.type_name() + ")");
  }
  else
  {
    impl = rhs.holder_;
    impl->owners.insert(holder_->owners.begin(), holder_->owners.end());
  }

  foreach(tendril* x, impl->owners)
        {
          x->holder_ = impl;
        }
  //set out connected state to true
  connected_ = true;
}

void tendril::disconnect()
{
  //release the holder
  holder_->release(this);
  //reset this holder_;
  holder_ = holder_->make(this);
  //set the connected_ flag to false
  connected_ = false;
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
  if (is_type<none> ())
  {
    holder_.reset(new holder<boost::python::object> (o, this));
  }
  else
  {
    holder_->setPython(o);
  }
}
}
