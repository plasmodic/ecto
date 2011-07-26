#include <gtest/gtest.h>
#include <ecto/ecto.hpp>


using ecto::tendril;
using ecto::spore;
TEST(SporeTest, LifeTime)
{
  {
    spore<double> d = tendril::make_tendril<double>();
    EXPECT_ANY_THROW(*d);
  }
  {
    spore<double> d;
    EXPECT_ANY_THROW(*d);
  }
  {
    tendril::ptr p = tendril::make_tendril<double>();
    spore<double> d = p; //p has to stay in scope...
    EXPECT_TRUE(d.p());
  }
}

TEST(SporeTest, NoDefault)
{
  tendril::ptr p = tendril::make_tendril<double>();
  spore<double> d = p; //p has to stay in scope...
  EXPECT_FALSE(d.user_supplied());
  EXPECT_FALSE(d.dirty());
  EXPECT_FALSE(d.has_default());

  d << 3.14;
  EXPECT_TRUE(d.dirty());
  EXPECT_TRUE(d.user_supplied());
  EXPECT_FALSE(d.has_default());

  //since the user already supplied a value this should be false...
  d.set_default_val(10);
  EXPECT_FALSE(d.has_default());

}

TEST(SporeTest, Default)
{
  tendril::ptr p = tendril::make_tendril<double>();
  EXPECT_FALSE(p->dirty());

  spore<double> d = p; //p has to stay in scope...
  EXPECT_FALSE(d.dirty());

  d.set_default_val(1.41421356);

  EXPECT_FALSE(d.user_supplied());
  EXPECT_FALSE(d.dirty());
  EXPECT_TRUE(d.has_default());

  EXPECT_EQ(*d, 1.41421356);
  EXPECT_FALSE(d.dirty());
  d.notify();
  EXPECT_FALSE(d.dirty());

  d << 3.14;
  EXPECT_TRUE(d.dirty());
  EXPECT_TRUE(d.user_supplied());
  EXPECT_TRUE(d.has_default());
}

template<typename T>
struct cbs
{
  cbs() : count(0), val(0) { }

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
  tendril::ptr p = tendril::make_tendril<double>();
  spore<double> d = p; //p has to stay in scope...
  d.set_default_val(1.41421356);

  cbs<double> c;
  d.set_callback(boost::ref(c));
  d.notify();
  EXPECT_EQ(c.count, 0);
  EXPECT_EQ(c.val, 0);

  d << 3.14;
  d.notify();
  EXPECT_EQ(c.count, 1);
  EXPECT_EQ(c.val, 3.14);
}

template <typename T, size_t size>
void printz(T(&array)[size])
{
  for(size_t i = 0; i < size; ++i)
  {
    std::cout << array[i] << std::endl;
  }
}

TEST(SporeTest, Enumeration)
{
  std::string Values[] =
  { "Hello", "No Way", "Howdy do?" };

  std::cout << sizeof(Values) / sizeof(std::string) << std::endl;
  printz(Values);

  tendril::ptr p = tendril::make_tendril<std::string>();
  spore<std::string> d = p;
}

TEST(SporeTest, Expressions)
{
  tendril::ptr ta = tendril::make_tendril<double>(),
    tb = tendril::make_tendril<double>(),
    tc = tendril::make_tendril<double>()
    ;

  spore<double> a(ta), b(tb), c(tc);
  a << 13.; 
  EXPECT_EQ(*a, 13.);

  b << 14.; 
  EXPECT_EQ(*b, 14.);

  c << 15.;
  EXPECT_EQ(*c, 15.);

  a << (*b + *c);
  
  EXPECT_EQ(*a, 29.);
  EXPECT_EQ(*b, 14.);
  EXPECT_EQ(*c, 15.);
  
}
