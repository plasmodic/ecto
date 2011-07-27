#include <gtest/gtest.h>
#include <ecto/ecto.hpp>

TEST(TendrilTest, Dirtiness)
{
  {
    ecto::tendril meh;
    EXPECT_FALSE(meh.dirty());
  }

  ecto::tendril t(0.5f, "docstring");

  EXPECT_EQ(t.type_name(), "float");
  EXPECT_EQ(t.doc(), "docstring");

  EXPECT_FALSE(t.dirty());
  EXPECT_EQ(t.get<float>(), 0.5f);
  EXPECT_FALSE(t.dirty());
  t << 0.75f;
  EXPECT_TRUE(t.dirty());
  EXPECT_EQ(t.get<float>(), 0.75f);
}

TEST(TendrilTest, Defaultness)
{
  {
    ecto::tendril meh;
    EXPECT_FALSE(meh.dirty());
    EXPECT_FALSE(meh.user_supplied());
    EXPECT_FALSE(meh.has_default());
  }

  {
    ecto::tendril meh(0.5f, "docstring");
    EXPECT_FALSE(meh.dirty());
    EXPECT_FALSE(meh.user_supplied());
    EXPECT_TRUE(meh.has_default());
    meh << 2.0f;
    EXPECT_TRUE(meh.has_default());
    EXPECT_TRUE(meh.user_supplied());
    EXPECT_TRUE(meh.dirty());
  }
  {
    ecto::tendril::ptr meh = ecto::tendril::make_tendril<float>();
    EXPECT_FALSE(meh->dirty());
    EXPECT_FALSE(meh->user_supplied());
    EXPECT_FALSE(meh->has_default());
    *meh << 2.0f;
    EXPECT_FALSE(meh->has_default());
    EXPECT_TRUE(meh->user_supplied());
    EXPECT_TRUE(meh->dirty());
  }
}

TEST(TendrilTest, NonPointerNess)
{
  ecto::tendril a(0.5f, "A float"), b, c;
  b = a;
  c = a;
  c << 3.14f;
  EXPECT_NE(a.get<float>(),c.get<float>());
  EXPECT_EQ(a.get<float>(),b.get<float>());
  EXPECT_NE(&a.get<float>(),&c.get<float>());
  EXPECT_NE(&a.get<float>(),&b.get<float>());
}

TEST(TendrilTest, Copyness)
{
  ecto::tendril a(0.5f, "A float"), b, c;
  b.copy_value(a);
  c.copy_value(b);
  c << 3.14f;
  EXPECT_NE(a.get<float>(),c.get<float>());
  EXPECT_EQ(a.get<float>(),b.get<float>());
  EXPECT_NE(&a.get<float>(),&c.get<float>());
  EXPECT_NE(&a.get<float>(),&b.get<float>());
  //self copy should be ok
  c.copy_value(a);

}

TEST(TendrilTest, Typeness)
{
  ecto::tendril a(0.5f, "A float"), b(0.5, "A double."), c;
  EXPECT_THROW(b.copy_value(a), ecto::except::TypeMismatch);
  EXPECT_NO_THROW(c = a);
  EXPECT_THROW(c.copy_value(b), ecto::except::TypeMismatch);
  EXPECT_NO_THROW(c = b);
  EXPECT_THROW(c.copy_value(a),ecto::except::TypeMismatch);
  ecto::tendril n1(ecto::tendril::none(),"A none"), n2(ecto::tendril::none(),"Another none"), n3;
  n2.copy_value(n1);
  n1.copy_value(n2);
  n1 = n2;
  n2 = n1;
  n1.copy_value(b);
  n2.copy_value(n1);
  EXPECT_EQ(n2.get<double>(),0.5);
  EXPECT_THROW(n2.copy_value(a), ecto::except::TypeMismatch);
  //EXPECT_THROW(b.copy_value(n3), ecto::except::ValueNone);


}

namespace bp = boost::python;
TEST(TendrilTest, BoostPyness)
{
  ecto::tendril bpt(bp::object(2.0), "A bp object");
  ecto::tendril dt(2.0, "A double");

  dt.copy_value(bpt);
  bpt.copy_value(dt);

  {
    ecto::tendril bpt(bp::object(), "A bp object");
    ecto::tendril dt(2.0, "A double");
    EXPECT_THROW(
        {
          dt.copy_value(bpt);
        }, ecto::except::TypeMismatch
   );
  }
  {
    ecto::tendril bpt(bp::object(), "A bp object"), bpt2(bp::object(), "another bp::object");
    EXPECT_FALSE( bpt.get<bp::object>());
  }
}

TEST(TendrilTest, BoostPyDefaultness)
{
  ecto::tendrils ts;
  ts.declare<boost::python::object>("x","A bp object");
  bp::object x;
  ts["x"] >> x;

  if(x == bp::object())
  {
    std::cout << "x is none" << std::endl;
  }
}

TEST(TendrilTest, SyntacticSugar)
{
  int x = 2;
  float y = 3.14;
  std::string z = "z";

  ecto::tendril tx(x,"doc"),ty(y,"doc"),tz(z,"doc");
  tz >> z;
  tz << z;
  ty >> y;
  ty << y;
  tx << x;
  tx >> x;
  EXPECT_THROW(tx >> y;,ecto::except::TypeMismatch);
  EXPECT_THROW(tx << z;,ecto::except::TypeMismatch);


  ecto::tendrils ts;
  ts.declare<int>("x");
  ts.declare<float>("y");
  ts.declare<std::string>("z");
  ts["x"] >> x;
  ts["x"] << x;
  ts["y"] >> y;
  ts["y"] << y;
  ts["z"] >> z;
  ts["z"] << z;
  EXPECT_THROW(ts["z"] >> y;,ecto::except::TypeMismatch);
  EXPECT_THROW(ts["z"] << x;,ecto::except::TypeMismatch);

  EXPECT_THROW(ts["w"] >> x;,std::runtime_error);
  EXPECT_THROW(ts["t"] << x;,std::runtime_error);

}

TEST(TendrilTest, Nones)
{
  ecto::tendril::ptr a = ecto::tendril::make_tendril<ecto::tendril::none>();
  ecto::tendril::ptr b = ecto::tendril::make_tendril<ecto::tendril::none>();
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
  a << b;
  std::cout << "a type: " << a->type_name() << "\n";
  std::cout << "b type: " << b->type_name() << "\n";
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
  a >> b;
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
}


TEST(TendrilTest, ConvertersCopied)
{
  ecto::tendril::ptr a = ecto::tendril::make_tendril<ecto::tendril::none>();
  ecto::tendril::ptr b = ecto::tendril::make_tendril<double>();
  EXPECT_FALSE(a->same_type(*b));
  EXPECT_FALSE(b->same_type(*a));
  *a = *b;
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
  boost::python::object obj(3.1415);
  *a << obj;
  EXPECT_EQ(a->get<double>(), 3.1415);
  boost::python::object obj2;
  *a >> obj2;
  boost::python::extract<double> e(obj2);
  EXPECT_TRUE(e.check());
  EXPECT_EQ(e(), 3.1415);
}
