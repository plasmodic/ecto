#include <ecto/python.hpp>
#include <gtest/gtest.h>

int main(int argc, char** argv)
{
  //Py_Initialize();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
