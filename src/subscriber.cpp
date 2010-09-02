#include <ros/util.hpp>
#include <ros/message.hpp>
#include <ros/queue.hpp>

namespace bip = boost::interprocess;

struct Point {
  float x, y, z;
};

int main(int argc, char** argv)
{

  queue<Point> q("somewhere", 24, bip::read_write);

  for (unsigned i=0; i< 100; ++i)
    {
      message_ptr<Point> msg = q.create();

    }

}


