template<typename T>
  const T& tendril::get() const
  {
    if (!impl_)
      throw std::logic_error("This connection is uninitialized! type: " + name_of<T> ());
    return *impl_base::get<T>(*impl_);
  }

template<typename T>
  T& tendril::get()
  {
    if (!impl_)
      throw std::logic_error("This connection is uninitialized! type: " + name_of<T> ());
    return *(impl_base::get<T>(*impl_));
  }

template<typename T>
  tendril tendril::make(const T& t, const std::string& name, const std::string& doc)
  {
    // fixme: allocators
    tendril c(impl_base::ptr(new impl<T> (t)));
    c.impl_->name = name;
    c.impl_->doc = doc;
    return c;
  }

#if NDEBUG
#define ECTO_ASSERT(_impl_check_ ) \
do {} while(false)
#else
#define ECTO_ASSERT(_impl_check_ ) \
do \
{ \
 _impl_check_; \
}while(false)
#endif
template<typename T>
  bool tendril::impl_base::check(tendril::impl_base& i)
  {
    //this fails across multiple modules!
    //return typeid(T) == i.type_info();
    //however type name should be ok?
    return std::strcmp(typeid(T).name(), i.type_info().name()) == 0;
  }

template<typename T>
  void tendril::impl_base::checkThrow(tendril::impl_base& i) throw (std::logic_error)
  {
    if (!check<T> (i))
      throw(std::logic_error(std::string(i.type_name() + " is not a " + name_of<T> ()).c_str()));
  }

template<typename T>
  T* tendril::impl_base::get(tendril::impl_base& i)
  {
    ECTO_ASSERT(checkThrow<T>(i));
    return reinterpret_cast<T*> (i.get());
  }
template<typename T>
  tendril::impl<T>::impl(const T& t) :
    t(t)
  {
  }

template<typename T>
  std::string tendril::impl<T>::type_name() const
  {
    return name_of<T> ();
  }
template<typename T>
  const std::type_info & tendril::impl<T>::type_info() const
  {
    return typeid(t);
  }

template<typename T>
  void* tendril::impl<T>::get()
  {
    return &t;
  }
template<typename T>
  void tendril::impl<T>:: setPython(boost::python::object o)
{
  boost::python::extract<T> get_T(o);
  if(get_T.check())
    t = get_T();
  else
    throw std::logic_error("Could not convert python object to type : " + type_name());
}
template<typename T>
boost::python::object tendril::impl<T>::getPython() const
{
//  boost::python::class_<T> c( "T", boost::python::init<>() );
//  boost::python::object o = c();
  return boost::python::object(t);

}
template<typename T>
  std::string tendril::impl<T>::value() const
  {
    std::stringstream ss;
    ss << t;
    return ss.str();//boost::lexical_cast<std::string>(t);
  }
