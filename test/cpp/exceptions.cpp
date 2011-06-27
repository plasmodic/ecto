#include <gtest/gtest.h>
#include <ecto/ecto.hpp>

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
    out.declare<std::string>("a");

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
  EXPECT_THROW(create_module<ExceptionalModule1>(), ecto::except::TypeMismatch);
}
TEST(Exceptions, ExceptionUnknownException)
{
  try
  {
    create_module<ExceptionUnknownException> ();
  } catch (except::EctoException& e)
  {
    std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
  }
  EXPECT_THROW(create_module<ExceptionUnknownException>(), ecto::except::EctoException);
}

TEST(Exceptions, ProcessException)
{
  module::ptr m = create_module<ProcessException> ();
  EXPECT_THROW(
      try
      {
        m->process();
      }
      catch (except::EctoException& e)
      {
        std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
        throw e;
      }
      ,
      ecto::except::EctoException);
}

TEST(Exceptions, NotExist)
{
  module::ptr m = create_module<NotExist> ();
  EXPECT_THROW(
      try
      {
        m->process();
      }
      catch (except::EctoException& e)
      {
        std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
        throw e;
      }
      ,
      ecto::except::EctoException);
}

TEST(Exceptions, WrongType)
{
  module::ptr m = create_module<WrongType> ();
  EXPECT_THROW(
      try
      {
        m->process();
      }
      catch (except::EctoException& e)
      {
        std::cout << "Good, threw an exception:\n" << e.what() << std::endl;
        throw e;
      }
      ,
      ecto::except::EctoException);
}
