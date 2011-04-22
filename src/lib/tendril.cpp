#include <ecto/tendril.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
namespace ecto
{
  tendril::tendril() :
    //impl_ is never not initialized
        impl_(new impl<none> (none(),this)),connected_(false)
  {
  }

  tendril::~tendril()
  {
    //SHOW();
    impl_->owners.erase(this);
  }

  tendril::tendril(const tendril& rhs):impl_(rhs.impl_),connected_(rhs.connected_)
  {
    impl_->owners.insert(this);
  }
  tendril& tendril::operator=(const tendril& rhs)
  {
    if(this == &rhs)
      return *this;

    impl_->owners.erase(this);
    impl_ = rhs.impl_;
    impl_->owners.insert(this);
    connected_ = rhs.connected_;
    return *this;
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
    if(connected_)
      throw std::runtime_error("Already connected. This is considered an error.");

    //check if the types aren't the same
    if (type_name() != rhs.type_name())
    {
      //they may be boost python types, attempt to steal
      //these only work argument is a boost python object
      //this will change the type of rhs to the type of impl_ if possible
      if (!impl_->steal(rhs) && !rhs.impl_->steal(*this))
      {
        throw std::runtime_error("bad connect! input(" + impl_->type_name() + ") != output(" + rhs.type_name() + ")");
      }
    }
    //propagate the new impl to all owners
    std::set<tendril*> owners = impl_->owners;
    foreach(tendril* x, owners)
    {
      x->impl_ = rhs.impl_;
    }
    //insert our old owners
    impl_->owners.insert(owners.begin(),owners.end());
    //set out connected state to true
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
      impl_.reset(new impl<boost::python::object> (o,this));
    }
    else
    {
      impl_->setPython(o);
    }
  }
}
