#include <gtest/gtest.h>
#include <ecto/ecto.hpp>

using ecto::tendril;
using ecto::spore;
TEST(SporeTest, LifeTime)
{
  {
    spore<double> d = tendril::make_tendril<double>();
    EXPECT_ANY_THROW(*d);
    EXPECT_ANY_THROW(d());
  }
  {
    spore<double> d;
    EXPECT_ANY_THROW(*d);
    EXPECT_ANY_THROW(d());
  }
  {
    tendril::ptr p = tendril::make_tendril<double>();
    spore<double> d = p; //p has to stay in scope...
    EXPECT_TRUE(d.p());
  }
}

TEST(SporeTest, NoDefault)
{

  {
    tendril::ptr p = tendril::make_tendril<double>();
    spore<double> d = p; //p has to stay in scope...
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

TEST(SporeTest, Default)
{
  {
    tendril::ptr p = tendril::make_tendril<double>();
    spore<double> d = p; //p has to stay in scope...
    d.set_default_val(1.41421356);

    EXPECT_FALSE(d.user_supplied());
    EXPECT_FALSE(d.dirty());
    EXPECT_TRUE(d.has_default());

    EXPECT_EQ(d(), 1.41421356);
    EXPECT_FALSE(d.dirty());

    EXPECT_EQ(*d, 1.41421356);
    EXPECT_TRUE(d.dirty());
    d.notify();
    EXPECT_FALSE(d.dirty());

    *d = 3.14;
    EXPECT_TRUE(d.dirty());
    EXPECT_TRUE(d.user_supplied());
    EXPECT_TRUE(d.has_default());
    EXPECT_EQ(d(), 3.14);

  }
}

template<typename T>
struct cbs
{
  cbs() :
      count(0), val(0)
  {
  }
  void operator()(const T& new_val)
  {
    val = new_val;
    count++;
  }
  int count;
  T val;
};

TEST(SporeTest, Callbacks)
{
  {
    tendril::ptr p = tendril::make_tendril<double>();
    spore<double> d = p; //p has to stay in scope...
    d.set_default_val(1.41421356);

    cbs<double> c;
    d.set_callback(boost::ref(c));
    d.notify();
    EXPECT_EQ(c.count, 0);
    EXPECT_EQ(c.val, 0);

    *d = 3.14;
    d.notify();
    EXPECT_EQ(c.count, 1);
    EXPECT_EQ(c.val, 3.14);
  }
}
