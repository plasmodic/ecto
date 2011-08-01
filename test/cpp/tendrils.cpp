#include <gtest/gtest.h>
#include <ecto/ecto.hpp>

using namespace ecto;

TEST(TendrilMap, Const)
{
  tendrils t;
  t.declare<bool>("bool", "booly an", true);

  tendril::ptr tp = t["bool"];
  EXPECT_TRUE(tp);
}

TEST(TendrilMap, Reference)
{
  tendrils t;
  t.declare<bool>("bool", "booly an", true);
  t.declare<bool>("b2");
  t.declare<std::string>("foo","A str", "bar");
  try
  {
    t["bool"] = t["foo"];
    ASSERT_FALSE(true);//should not reach here.
  }catch(std::exception& e)
  {
    EXPECT_TRUE(std::string(e.what()) == "bool may not be reassigned to std::string");
  }
  EXPECT_TRUE(t["bool"]->is_type<bool>());
  t.get<bool>("b2") = false;
  t.get<bool>("bool") = true;
  t["b2"] = t["bool"];
  EXPECT_TRUE(t.get<bool>("b2"));
  t.get<bool>("b2") = false;
  EXPECT_FALSE(t.get<bool>("b2"));
  EXPECT_FALSE(t.get<bool>("bool"));

}


TEST(TendrilMap, CopyValue)
{
  tendrils t1,t2;
  t1.declare<bool>("a", "booly an", false);
  t1.declare<tendril::none>("any","a nony");
  t2.declare<bool>("b","a booly an", true);
  t1["a"] << t2["b"];
  t1["any"] << t2["b"];
  t1["any"] << t1["a"];
  EXPECT_TRUE(t1.get<bool>("any"));

}
