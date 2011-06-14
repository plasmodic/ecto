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

