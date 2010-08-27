#include <ros/util.hpp>
#include <ros/message.hpp>
#include <ros/topic.hpp>


struct Point {
  float x, y, z;
};

int main(int argc, char** argv)
{

  std::cout << name_of<int>() << "\n";

  topic<Point> t;

  message<Point> msg = t.create();


}


