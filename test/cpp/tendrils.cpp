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


