
#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>

struct tendril
{
  struct holder_base 
  {
    virtual ~holder_base();

    virtual void set(void* value, void* module) = 0;

  };

  template <typename T, typename U>
  struct holder : holder_base
  {
    T value;
    typedef boost::function<void(U*,T)> onchange_t;

    onchange_t fn;

    void set(void* value, void* m)
    {
      T* realval = static_cast<T*>(value);
      U* module = static_cast<U*>(m);

      fn(module, *realval);
    }
  };

  template <typename T>
  struct holder<T, void> : holder_base
  {
    T value;
    typedef boost::function<void(T)> onchange_t;

    onchange_t fn;

    void set(void* value)
    {
      T* realval = static_cast<T*>(value);

      fn(*realval);
    }
  };


  // tendril(T t, boost::function<void(T)>) // non memberfunction case

  template <typename T, typename U>
  tendril(T t, void(U::*fn)(T))
  {
    std::cout << __PRETTY_FUNCTION__ << "\n";

    holder<T, U>* newptr(new holder<T, U>);
    newptr->value = t;
    newptr->fn = fn;
    ptr.reset(newptr);
  }
  
  tendril() { }

  boost::shared_ptr<holder_base> ptr;

  template <typename T, typename U>
  void set(T newvalue, U* module)
  {
    std::cout << __PRETTY_FUNCTION__ << "\n";
    ptr->set(&newvalue, module);

  }

};

tendril::holder_base::~holder_base() { }

struct module
{
  float f;
  std::string g;
  tendril t, u;
  
  void f_changed(float newval) {  
    std::cout << "setting new value to " << newval << "\n";
    f = newval; 
  }

  static void g_changed(std::string newval) {
    std::cout << "setting g to " << newval << "\n";
  }

  static void declare_param(tendril& t, tendril& u)
  {
    t = tendril(3.14159f, &module::f_changed);
    //u = tendril(std::string("DEFAULT"), &cell::g_changed);
  }

  void set_callbacks() {
    
  }

  //  cell() : {
  //
  //  }

};

struct S {
  static void foo() { std::cout << __PRETTY_FUNCTION__ << "\n";}
};

struct NoFoo {

};

/*
template <typename T, typename U = void>
struct has_foo
{
  enum { value = 0 };
};

template <typename T>
struct has_foo<T, typename T::foo>
{
  enum { value = 1 };
};
*/

template <class T>
struct has_foo
{
  typedef char yes;
  typedef char (&no)[2];

  template <long I> struct S { };

  // SFINAE eliminates this when the type of arg is invalid
  template <class U>
  static yes test(S<sizeof(&U::foo)>);

  // overload resolution prefers anything at all over "..."
  template <class U>
  static no test(...);

  void existent_fn();
  const static S<sizeof(&has_foo<T>::existent_fn)> sarg;

  // see which overload is chosen when U == T
  enum { value = sizeof(test<T>(sarg)) == sizeof(yes) };

  //  typedef mpl::bool_<value> type;
};


struct m1 {
  void foo() { std::cout << __PRETTY_FUNCTION__ << "\n"; }
};

struct m2 { };


struct base 
{
  virtual void foo() = 0;
};

template <int I>
struct int_ { };


template <typename T>
struct thunk : base 
{
  T t;

  void foo(int_<0>, T& t) { std::cout << __PRETTY_FUNCTION__ << " DEFAULT FOO\n";  }
  void foo(int_<1>, T& t) { std::cout << __PRETTY_FUNCTION__ << "\n"; t.foo(); };

  void foo() 
  {
    this->foo(int_<has_foo<T>::value>(), t);
  }
  
};




int main(int, char**)
{
  std::cout << has_foo<m1>::value << "\n";
  std::cout << has_foo<m2>::value << "\n";
  
  thunk<m1> tm1;
  base& bm1 = tm1;

  thunk<m2> tm2;
  base& bm2 = tm2;

  bm1.foo();
  bm2.foo();


  std::cout << __PRETTY_FUNCTION__ << "\n";
  tendril t, u;
  //cell::declare_param(t);
  module m;

  t.set(777.777f, &m);
  //u.set(std::string("SETVALUE"));



}
