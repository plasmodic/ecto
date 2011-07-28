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

TEST(TendrilTest, Constructors)
{
  {
    ecto::tendril meh;
    EXPECT_FALSE(meh.dirty());
    EXPECT_FALSE(meh.user_supplied());
    EXPECT_FALSE(meh.has_default());
    EXPECT_TRUE(meh.is_type<ecto::tendril::none>());
  }

  {
    ecto::tendril meh(0.5f, "docstring");
    EXPECT_FALSE(meh.dirty());
    EXPECT_FALSE(meh.user_supplied());
    EXPECT_TRUE(meh.has_default());
    EXPECT_TRUE(meh.is_type<float>());

    meh << 2.0f;

    EXPECT_TRUE(meh.has_default());
    EXPECT_TRUE(meh.user_supplied());
    EXPECT_TRUE(meh.dirty());
    EXPECT_TRUE(meh.is_type<float>());
  }
  {
    ecto::tendril::ptr meh = ecto::make_tendril<float>();
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
  b << a;
  c << b;
  c << 3.14f;
  EXPECT_NE(a.get<float>(),c.get<float>());
  EXPECT_EQ(a.get<float>(),b.get<float>());
  EXPECT_NE(&a.get<float>(),&c.get<float>());
  EXPECT_NE(&a.get<float>(),&b.get<float>());
  //self copy should be ok
  c << a;

}

TEST(TendrilTest, Typeness)
{
  ecto::tendril a(0.5f, "A float"), b(0.5, "A double."), c;
  EXPECT_THROW(b << a, ecto::except::TypeMismatch);
  EXPECT_NO_THROW(c = a);
  EXPECT_THROW(c << b, ecto::except::TypeMismatch);
  EXPECT_NO_THROW(c = b);
  EXPECT_THROW(c << a,ecto::except::TypeMismatch);
  ecto::tendril n1(ecto::tendril::none(),"A none"), n2(ecto::tendril::none(),"Another none"), n3;
  n2 << n1;
  n1 << n2;
  n1 = n2;
  n2 = n1;
  n1 << b;
  n2 << n1;
  EXPECT_EQ(n2.get<double>(),0.5);
  EXPECT_THROW(n2 << a, ecto::except::TypeMismatch);
  //EXPECT_THROW(b.copy_value(n3), ecto::except::ValueNone);
}

TEST(TendrilTest, AssignmentOfPODS)
{
  ecto::tendril a(0.005f, "A float");
  EXPECT_TRUE(a.is_type<float>());
  EXPECT_EQ(a.get<float>(), 0.005f);

  ecto::tendril b(500.0, "some double");
  EXPECT_TRUE(b.is_type<double>());
  EXPECT_EQ(b.get<double>(), 500.0);

  ecto::tendril c;
  EXPECT_TRUE(c.is_type<ecto::tendril::none>());

  // nothing is mutated on throwing conversion
  EXPECT_THROW(b << a, ecto::except::TypeMismatch);
  EXPECT_TRUE(a.is_type<float>());
  EXPECT_EQ(a.get<float>(), 0.005f);
  EXPECT_TRUE(b.is_type<double>());
  EXPECT_EQ(b.get<double>(), 500.0);

  // assignee takes on the type and value of the assigned 
  EXPECT_NO_THROW(c = a);
  EXPECT_TRUE(a.is_type<float>());
  EXPECT_EQ(a.get<float>(), 0.005f);
  EXPECT_TRUE(c.is_type<float>());
  EXPECT_EQ(c.get<float>(), 0.005f);

  // again nothing is mutated on throwing conversion
  EXPECT_THROW(c << b, ecto::except::TypeMismatch);
  EXPECT_TRUE(b.is_type<double>());
  EXPECT_EQ(b.get<double>(), 500.0);
  EXPECT_TRUE(c.is_type<float>());
  EXPECT_EQ(c.get<float>(), 0.005f);

  // again assignment suceeds
  EXPECT_NO_THROW(c = b);
  EXPECT_TRUE(b.is_type<double>());
  EXPECT_EQ(b.get<double>(), 500.0);
  EXPECT_TRUE(c.is_type<double>());
  EXPECT_EQ(c.get<double>(), 500.0);

  // same as above
  EXPECT_THROW(c << a,ecto::except::TypeMismatch);
}

TEST(TendrilTest, AssignmentOfNone)
{
  ecto::tendril n1(ecto::tendril::none(),"A none");
  ecto::tendril n2(ecto::tendril::none(),"Another none");
  
  ecto::tendril n3;
  n2 << n1;
  EXPECT_EQ(n1.get<ecto::tendril::none>(), n2.get<ecto::tendril::none>());
  n1 << n2;
  EXPECT_EQ(n1.get<ecto::tendril::none>(), n2.get<ecto::tendril::none>());

  n1 = n2;
  n2 = n1;

  ecto::tendril _500(500.0, "five hundred");

  n1 << _500;
  EXPECT_TRUE(n1.is_type<double>());
  EXPECT_TRUE(_500.is_type<double>());
  EXPECT_EQ(n1.get<double>(), 500.0);
  EXPECT_EQ(_500.get<double>(), 500.0);
  
}

namespace bp = boost::python;
TEST(TendrilTest, Python2PODConversion)
{
  ecto::tendril bpt(bp::object(2.05), "A bp object");
  ecto::tendril dt(7.05, "A double");

  // tendril(bp::object) autoconverts from tendril(nonobject)
  dt << bpt;
  EXPECT_EQ(2.05, dt.get<double>());

  // can be overwritten thereafter
  dt << 7.05;
  EXPECT_EQ(dt.get<double>(), 7.05);

  // copyee is not mutated
  double value = bp::extract<double>(bpt.get<bp::object>());
  EXPECT_EQ(value, 2.05);

  // dt has not lost its internal type
  EXPECT_THROW( dt << std::string("NOTADOUBLE"), ecto::except::TypeMismatch);

  // but we can't autoconvert from None
  bpt << bp::object();
  EXPECT_THROW(dt << bpt, ecto::except::TypeMismatch);
}

TEST(TendrilTest, POD2PythonConversion)
{
  ecto::tendril bpt(bp::object(2.05), "A bp object");
  ecto::tendril dt(7.05, "A double");

  EXPECT_EQ(bp::extract<double>(bpt.get<bp::object>()), 2.05);
  EXPECT_TRUE(bpt.is_type<bp::object>());

  bpt << dt;

  // bpt is still a bp::object
  EXPECT_TRUE(bpt.is_type<bp::object>());

  // dt is still a double
  EXPECT_TRUE(dt.is_type<double>());

  // double was copied correctly into dt
  EXPECT_EQ(bp::extract<double>(bpt.get<bp::object>()), 7.05);

  // dt was not mutated
  EXPECT_EQ(dt.get<double>(), 7.05);

}

TEST(TendrilTest, BoostPyDefaultness)
{
  ecto::tendrils ts;
  ts.declare<bp::object>("x","A bp object");
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
  using ecto::tendril;
  using ecto::make_tendril;

  tendril::ptr a = make_tendril<tendril::none>();
  tendril::ptr b = make_tendril<tendril::none>();
  EXPECT_TRUE(a->is_type<tendril::none>());
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
  a << b;
  std::cout << "a type: " << a->type_name() << "\n";
  std::cout << "b type: " << b->type_name() << "\n";
  EXPECT_TRUE(a->is_type<tendril::none>());
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
  a >> b;
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));

  // you can assign anything to a none tendril, it changes type
  a << 7.05;
  EXPECT_TRUE(a->is_type<double>());
  EXPECT_EQ(a->get<double>(), 7.05);

  // note: now a is a double, you can't assign a string to it
  std::string s("ess");
  EXPECT_THROW(a << s, ecto::except::TypeMismatch);

  // assignment makes it a vanilla none again
  a = b;
  EXPECT_TRUE(a->is_type<tendril::none>());
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));

  // bp object with a string in it
  bp::object obj(s);
  
  a << obj;
  EXPECT_TRUE(a->is_type<bp::object>());
}

TEST(TendrilTest, ConversionTableFromNoneColumn)
{
  using ecto::tendril;
  tendril none_;

  { // none << none
    tendril othernone_;
    othernone_ << none_;
  }

  { // object << none
    tendril object_(bp::object(3.14159), "pyobj");
    EXPECT_THROW(object_ << none_, ecto::except::ValueNone);
  }

  { // double << none
    tendril double_(3.14159, "double");
    EXPECT_THROW(double_ << none_, ecto::except::ValueNone);
  }
}


TEST(TendrilTest, ConversionTableFromPyObjectColumn)
{
  using ecto::tendril;
  tendril pypi_(bp::object(3.1415), "py pi");

  { // none << object
    tendril none_;
    none_ << pypi_;
    bp::object rt = none_.get<bp::object>();
    EXPECT_EQ(bp::extract<double>(rt), 3.1415);
  }

  { // object << object
    tendril o2(bp::object(7.777), "sevens");
    o2 << pypi_;
    bp::object rt = o2.get<bp::object>();
    EXPECT_EQ(bp::extract<double>(rt), 3.1415);
  }

  { // double << object (compatible)
    tendril double_(5.555, "double");
    double_ << pypi_;
    EXPECT_EQ(double_.get<double>(), 3.1415);
  }

  { // double << object (incompatible)
    tendril string_(std::string("oops"), "double");
    EXPECT_THROW(string_ << pypi_, ecto::except::TypeMismatch);
  }
}

TEST(TendrilTest, ConversionTableFromUDTColumn)
{
  using ecto::tendril;
  tendril udt_(std::string("STRINGY"), "py pi");

  { // none << udt
    tendril none_;
    none_ << udt_;
    std::string s = none_.get<std::string>();
    EXPECT_EQ(s, "STRINGY");
  }

  { // object << udt
    tendril o2(bp::object(7.777), "sevens");
    o2 << udt_;
    bp::object rt = o2.get<bp::object>();
    std::string xtracted = bp::extract<std::string>(rt);
    EXPECT_EQ(xtracted, std::string("STRINGY"));
  }

  { // string << udt (compatible)
    tendril string_(std::string("NOTSTRINGY"), "is other string");
    string_ << udt_;
    EXPECT_EQ(string_.get<std::string>(), std::string("STRINGY"));
    // not the same string
    EXPECT_NE(&(string_.get<std::string>()), &(udt_.get<std::string>()));
  }

  { // double << udt (incompatible)
    tendril double_(3.1415, "double");
    EXPECT_THROW(double_ << udt_, ecto::except::TypeMismatch);
  }
}


TEST(TendrilTest, ConvertersCopied)
{
  ecto::tendril::ptr a = ecto::make_tendril<ecto::tendril::none>();
  ecto::tendril::ptr b = ecto::make_tendril<double>();
  EXPECT_FALSE(a->same_type(*b));
  EXPECT_FALSE(b->same_type(*a));
  *a = *b;
  EXPECT_TRUE(a->same_type(*b));
  EXPECT_TRUE(b->same_type(*a));
  bp::object obj(3.1415);
  *a << obj;
  EXPECT_EQ(a->get<double>(), 3.1415);
  bp::object obj2;
  *a >> obj2;
  bp::extract<double> e(obj2);
  EXPECT_TRUE(e.check());
  EXPECT_EQ(e(), 3.1415);
}
