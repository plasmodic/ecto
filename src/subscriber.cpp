#include <ros/util.hpp>
#include <ros/message.hpp>
#include <ros/queue.hpp>

namespace bip = boost::interprocess;

struct Point {
  float x, y, z;
};

int main(int argc, char** argv)
{

  std::cout << name_of<int>() << "\n";

  queue<Point> q("somewhere", 24, bip::read_write);

  for (unsigned i=0; i< 100; ++i)
    {
      message<Point> msg = q.create();

    }

}


