#include <gtest/gtest.h>
#include <ecto/ecto.hpp>

using ecto::tendril;
using ecto::spore;
TEST(SporeTest, LifeTime)
{
  {
    spore<double> d = tendril::make_tendril<double>();
    EXPECT_TRUE(!d.p());
  }
  {
    spore<double> d;
    EXPECT_TRUE(!d.p());
  }
  {
    tendril::ptr p = tendril::make_tendril<double>();
    spore<double> d = p;//p has to stay in scope...
    EXPECT_TRUE(d.p());
  }
}
TEST(SporeTest, Interface)
{

  {
    tendril::ptr p = tendril::make_tendril<double>();
    spore<double> d = p;//p has to stay in scope...
    EXPECT_FALSE(d.user_supplied());
    EXPECT_FALSE(d.dirty());
    EXPECT_FALSE(d.has_default());

    *d = 3.14;
    EXPECT_TRUE(d.dirty());
    EXPECT_TRUE(d.user_supplied());
    EXPECT_FALSE(d.has_default());

    //since the user already supplied a value this should be false...
    d.set_default_val(10);
    EXPECT_FALSE(d.has_default());
  }
}
