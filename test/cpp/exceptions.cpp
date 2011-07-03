#include <gtest/gtest.h>
#include <ecto/ecto.hpp>

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
  EXPECT_THROW(create_cell<ExceptionalModule1>(), ecto::except::TypeMismatch);
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
