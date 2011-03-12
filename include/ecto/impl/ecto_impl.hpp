template <typename T>
const T& connection::get() const
{
  //SHOW();
  // fixme: typecheck
  return *impl_base::get<T>(*impl_);
}

template <typename T>
T& connection::get()
{
  // fixme: typecheck
  //SHOW();
  return *(impl_base::get<T>(*impl_));
}

template <typename T>
connection connection::make(const T& t,const std::string& name, const std::string& doc)
{
  // fixme: allocators
  connection c(impl_base::ptr(new impl<T>(t)));
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
bool connection::impl_base::check(connection::impl_base& i)
{
  return typeid(T) == i.type_info();
}

template<typename T>
void connection::impl_base::checkThrow(connection::impl_base& i) throw(std::logic_error)
{
  if(!check<T>(i))
    throw(std::logic_error(std::string(i.type_name() + " is not a " + name_of<T>()).c_str()));
}

template<typename T>
T* connection::impl_base::get(connection::impl_base& i)
{
  ECTO_ASSERT(checkThrow<T>(i));
  return reinterpret_cast<T*>(i.get());
}
template<typename T>
connection::impl<T>::impl(const T& t) :
                          t(t){}

template<typename T>
std::string
connection::impl<T>::type_name() const
{
    return name_of<T>();
}
template<typename T>
const std::type_info & connection::impl<T>::type_info() const
{
  return typeid(t);
}

template<typename T>
void* connection::impl<T>::get() { return &t; }

template<typename T>
std::string connection::impl<T>::value() const
{
  return boost::lexical_cast<std::string>(t);
}

template<typename T>
connection& module::setOut(const std::string& name,
                      const std::string& doc,
                      const T& t)
{
  outputs[name] = ecto::connection::make<T>(t,name,doc);
  return outputs[name];
}
template<typename T>
connection& module::setIn(const std::string& name,
                      const std::string& doc,
                      const T& t)
{
  inputs[name] = ecto::connection::make<T>(t,name,doc);
  return inputs[name];
}

template<typename T>
T& module::getOut(const std::string& name)
{
  //FIXME check for null, throw
  return outputs[name].get<T>();
}
template<typename T>
const T& module::getIn(const std::string& name)
{
  //FIXME check for null, throw
  return inputs[name].get<T>();
}
