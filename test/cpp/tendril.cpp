#include <gtest/gtest.h>
#include <ecto/ecto.hpp>
#include <ecto/tendril.hpp>

TEST(TendrilTest, Dirtiness)
{
  {
    ecto::tendril meh;
    EXPECT_FALSE(meh.dirty());
  }

  ecto::tendril t(0.5f, "docstring");

  EXPECT_FALSE(t.dirty());
  EXPECT_EQ(t.doc(), "docstring");
  EXPECT_EQ(t.get<float>(), 0.5f);
  EXPECT_EQ(t.type_name(), "float");
  t.get<float>() = 0.75f;
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

//TEST(TendrilTest, Raw)
//{
//  ecto::
//  ecto::tendril t();
//
//}
