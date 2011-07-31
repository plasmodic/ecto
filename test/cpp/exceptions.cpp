#include <gtest/gtest.h>
#include <ecto/ecto.hpp>
#include <ecto/plasm.hpp>
#include <ecto/scheduler/threadpool.hpp>

#define STRINGDIDLY(A) std::string(#A)

using namespace ecto;
struct ExceptionalModule1
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<double> ("d");
    p.declare<float> ("f").set_default_val(p.get<float> ("d"));
  }
};

struct ExceptionUnknownException
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<double> ("d");
  }
  static void
  declare_io(const tendrils& p, tendrils& in, tendrils& out)
  {
    in.declare<double> ("d");
    throw "A string";
  }
};

struct NotExist
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<int> ("a");
  }
  static void
  declare_io(const tendrils& p, tendrils& in, tendrils& out)
  {
    in.declare<double> ("d");
    in.declare<ExceptionalModule1> ("c");
    in.declare<std::string> ("e");
    out.declare<std::string> ("a");
  }

  int
  process(tendrils& in, tendrils& out)
  {
    in.get<double> ("a");
    return 0;
  }
};

struct WrongType
{
  static void
  declare_io(const tendrils& p, tendrils& in, tendrils& out)
  {
    in.declare<double> ("d");
  }
  int
  process(tendrils& in, tendrils& out)
  {
    in.get<int> ("d");
    return 0;
  }
};

struct ParameterCBExcept
{
  static void
  declare_params(tendrils& p)
  {
    p.declare<double> ("x");
  }
  void xcb(double x)
  {
    std::cout << "called back***" << std::endl;
    throw std::runtime_error("I'm a bad callback, and I like it that way.");
  }
  void
  configure(tendrils& p,tendrils& in, tendrils& out)
  {
    std::cout << "configurated ***" << std::endl;
    spore<double> x = p["x"];
    x.set_callback(boost::bind(&ParameterCBExcept::xcb,this,_1));
  }
};

struct ProcessException
{
  int
  process(tendrils& in, tendrils& out)
  {
    throw std::logic_error("A standard exception");
    return ecto::OK;
  }
};

TEST(Exceptions, ExceptionalModules)
{
  try
  {
    create_cell<ExceptionalModule1> ();
  } catch (except::EctoException& e)
  {
    std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
  }
  EXPECT_THROW(create_cell<ExceptionalModule1>(), ecto::except::EctoException);
}
TEST(Exceptions, ExceptionUnknownException)
{
  try
  {
    create_cell<ExceptionUnknownException> ();
  } catch (except::EctoException& e)
  {
    std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
  }
  EXPECT_THROW(create_cell<ExceptionUnknownException>(), ecto::except::EctoException);
}

#define MEH(x, y) x

TEST(Exceptions, ProcessException)
{
  std::string stre("Original Exception: std::logic_error\n"
    "  What   : A standard exception\n"
    "  Module : ProcessException\n"
    "  Function: process");
  cell::ptr m = create_cell<ProcessException> ();
  EXPECT_THROW(
      try
      {
        m->process();
      }
      catch (except::EctoException& e)
      {
        std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
        if(stre != e.msg_)
        {
          throw std::runtime_error("Got :" + e.msg_ +"\nExpected :" +stre);
        }
        throw e;
      }
      ,
      ecto::except::EctoException);
}

TEST(Exceptions, NotExist)
{
  std::string
      stre(
           "'a' does not exist in this tendrils object. Possible keys are:  'c':type(ExceptionalModule1) 'd':type(double) 'e':type(std::string)\n"
             "  Hint   : 'a' does exist in parameters (type == int) outputs (type == std::string)\n"
             "  Module : NotExist\n"
             "  Function: process");

  cell::ptr m = create_cell<NotExist> ();
  EXPECT_THROW(
      try
      {
        m->process();
      }
      catch (except::EctoException& e)
      {
        std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
        if(stre != e.msg_)
        {
          throw std::runtime_error("Got :" + e.msg_ +"\nExpected :" +stre);
        }
        throw e;
      }
      ,
      ecto::except::EctoException);
}

TEST(Exceptions, WrongType)
{
  std::string stre("double is not a int\n"
"  Hint : 'd' is of type: double\n"
"  Module : WrongType\n"
"  Function: process");
  cell::ptr m = create_cell<WrongType> ();
  EXPECT_THROW(
      try
      {
        m->process();
      }
      catch (except::EctoException& e)
      {
        std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
        if(stre != e.msg_)
        {
          throw std::runtime_error("Got :" + e.msg_ +"\nExpected :" +stre);
        }
        throw e;
      }
      ,
      ecto::except::EctoException);
}

TEST(Exceptions, WrongType_sched)
{
  std::string stre("double is not a int\n"
"  Hint : 'd' is of type: double\n"
"  Module : WrongType\n"
"  Function: process");
  cell::ptr m = create_cell<WrongType> ();
  plasm::ptr p(new plasm);
  p->insert(m);
  scheduler::threadpool sched(p);
  EXPECT_THROW(
      try
      {
        sched.execute(8,1);
      }
      catch (except::EctoException& e)
      {
        std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
        if(stre != e.msg_)
        {
          throw std::runtime_error("Got :" + e.msg_ +"\nExpected :" +stre);
        }
        throw e;
      }
      ,
      ecto::except::EctoException);
}

TEST(Exceptions, ParameterCBExcept_sched)
{
  std::string stre("Original Exception: std::runtime_error\n"
"  What   : I'm a bad callback, and I like it that way.\n"
"  Module : ParameterCBExcept\n"
"  Function: Parameter Callback for 'x'"
);
  cell::ptr m = create_cell<ParameterCBExcept> ();
  m->parameters["x"] << 5.1;
  m->parameters["x"]->dirty(true);
  plasm::ptr p(new plasm);
  p->insert(m);
  scheduler::threadpool sched(p);
  EXPECT_THROW(
      try
      {
        sched.execute(8,1);
      }
      catch (except::EctoException& e)
      {
        std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
        if(stre != e.msg_)
        {
          throw std::runtime_error("Got :" + e.msg_ +"\nExpected :" +stre);
        }
        throw e;
      }
      ,
      ecto::except::EctoException);
}
