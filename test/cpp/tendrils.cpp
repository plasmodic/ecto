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
//  tendrils t;
//  t.declare<bool>("bool", "booly an", true);
//  t.declare<bool>("b2");
//  t.declare<std::string>("foo","A str", "bar");
//  try
//  {
//    t.declare("bool",t["foo"]);
//    ASSERT_FALSE(true);//should not reach here.
//  }catch(std::exception& e)
//  {
//    EXPECT_TRUE(std::string(e.what()) == "bool may not be reassigned to std::string");
//  }
//  EXPECT_TRUE(t["bool"]->is_type<bool>());
//  t.get<bool>("b2") = false;
//  t.get<bool>("bool") = true;
//  t.declare("b2"] = t["bool"];
//  EXPECT_TRUE(t.get<bool>("b2"));
//  t.get<bool>("b2") = false;
//  EXPECT_FALSE(t.get<bool>("b2"));
//  EXPECT_FALSE(t.get<bool>("bool"));

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

TEST(tendrils, DefaultCtr)
{
  tendrils t;
  EXPECT_TRUE(t.size() == 0);

  tendrils::ptr tp(new tendrils());
  EXPECT_TRUE(tp->size() == 0);
}
//tendrils not copiable. good.
//TEST(tendrils, AssignementOperator)
//{
//  tendrils t1,t2;
//  t1.declare<int>("x","x is an int",3);
//  t1 = t2;
//  EXPECT_EQ(t2.get<int>("x"),3);
//  t2.get<int>("x") = 5;
//  EXPECT_EQ(t1.get<int>("x"),5);
//}

//test use of insert for copy
TEST(tendrils, Copy)
{
  tendrils t1,t2;
  t1.declare<int>("x","x is an int",3);
  t2.insert(t1.begin(),t1.end());
  EXPECT_EQ(t2.get<int>("x"),3);
  t2.get<int>("x") = 5;
  EXPECT_EQ(t1.get<int>("x"),5);
}


TEST(tendrils, Declare)
{
  tendrils t1;
  t1.declare<int>("x", "x is an int", 3);
  EXPECT_EQ(t1.get<int>("x"), 3);
  EXPECT_EQ(t1.size(), 1u);
  try
  {
    t1.declare<int>("x", "another declare", 11);
  } catch (std::exception& e)
  {
    EXPECT_NE(std::string(e.what()).find("You can't redeclare a tendril!"), std::string::npos);
  }

  //should still be valid after throw.
  EXPECT_EQ(t1.get<int>("x"), 3);

  //TODO: should you be able to do this?
  t1.clear();
  t1.declare<int>("x", "yet another declare", 17);
  EXPECT_EQ(t1.get<int>("x"), 17);
}

struct SpamFoo
{
  int blammo;
};

TEST(tendrils, SyntacticSugarness)
{
  tendrils t1, t2;
  t1.declare<int>("x", "x is an int", 3);
  t2.declare<std::string>("yy", "yy's doc");
  t2.declare<int>("y", "y is an int", 10);
  t1.declare<std::string>("s");
  t2.declare<SpamFoo>("f", "SpamFoo is stuffs");
  std::string s("howdy");
  t1["s"] << s;
  EXPECT_EQ(t1.get<std::string>("s"), "howdy");
  t1.get<std::string>("s") = "sally";
  t1["s"] >> s;
  EXPECT_EQ(s, "sally");
  s = "harry";
  t1["s"] << s;
  s = "";
  t2["yy"] << t1["s"];
  t2["yy"] >> s;
  EXPECT_EQ(s, "harry");
  tendril x(std::string("foobar"), "docstr");
  t2["yy"] << x;
  EXPECT_EQ(t2["yy"]->doc(), "yy's doc");
  EXPECT_EQ(t2.get<std::string>("yy"), "foobar");

  tendril::ptr xp = make_tendril<std::string>();
  xp << std::string("hello there.");
  t2["yy"] << xp;
  EXPECT_EQ(t2.get<std::string>("yy"), "hello there.");
  t1["s"] >> xp;
  EXPECT_EQ(xp->get<std::string>(), "harry");

  try
  {
    t1["s"] << t2["f"];
  } catch (std::exception& e)
  {
    EXPECT_NE(std::string(e.what()).find("std::string is not compatible with SpamFoo"), std::string::npos);
  }
  try
  {
    t1["s"] >> t2["f"];
  } catch (std::exception& e)
  {
    EXPECT_NE(std::string(e.what()).find("SpamFoo is not compatible with std::string"), std::string::npos);
  }
}


