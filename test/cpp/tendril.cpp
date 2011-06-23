#include <gtest/gtest.h>
#include <ecto/ecto.hpp>
TEST(TendrilTest, Dirtiness)
{
  {
    ecto::tendril meh;
    EXPECT_FALSE(meh.dirty());
  }

  ecto::tendril t(0.5f, "docstring");

  EXPECT_FALSE(t.dirty());
  EXPECT_EQ(t.doc(), "docstring");
  EXPECT_EQ(t.read<float>(), 0.5f);
  EXPECT_FALSE(t.dirty());
  EXPECT_EQ(t.type_name(), "float");
  t.get<float>() = 0.75f;
  EXPECT_TRUE(t.dirty());
  EXPECT_EQ(t.read<float>(), 0.75f);
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
    meh.get<float>() = 2.0f;
    EXPECT_TRUE(meh.has_default());
    EXPECT_TRUE(meh.user_supplied());
    EXPECT_TRUE(meh.dirty());
  }
  {
    ecto::tendril::ptr meh = ecto::tendril::make_tendril<float>();
    EXPECT_FALSE(meh->dirty());
    EXPECT_FALSE(meh->user_supplied());
    EXPECT_FALSE(meh->has_default());
    meh->get<float>() = 2.0f;
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
  c.get<float> () = 3.14;
  EXPECT_NE(a.read<float>(),c.read<float>());
  EXPECT_EQ(a.read<float>(),b.read<float>());
  EXPECT_NE(&a.read<float>(),&c.read<float>());
  EXPECT_NE(&a.read<float>(),&b.read<float>());
}

TEST(TendrilTest, Copyness)
{
  ecto::tendril a(0.5f, "A float"), b, c;
  b.copy_value(a);
  c.copy_value(b);
  c.get<float> () = 3.14;
  EXPECT_NE(a.read<float>(),c.read<float>());
  EXPECT_EQ(a.read<float>(),b.read<float>());
  EXPECT_NE(&a.read<float>(),&c.read<float>());
  EXPECT_NE(&a.read<float>(),&b.read<float>());
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
        }, ecto::except::ValueNone);
  }
  {
    ecto::tendril bpt(bp::object(), "A bp object"), bpt2(bp::object(), "another bp::object");
    EXPECT_TRUE( bpt.get<bp::object>() == bp::object());
  }

}
