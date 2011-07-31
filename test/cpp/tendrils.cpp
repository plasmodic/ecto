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
  t.declare<std::string>("foo","A str", "bar");
  t["bool"].reset();
  EXPECT_TRUE(t["bool"]); //not by reference
  t["bool"] = t["foo"];
  EXPECT_TRUE(t["bool"]->is_type<bool>());
}


