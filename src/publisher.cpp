#include <ros/util.hpp>
#include <ros/message.hpp>
#include <ros/queue.hpp>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost;

namespace ip = boost::interprocess;

struct Point {
  float x, y, z;
};

int main(int argc, char** argv)
{

  asio::io_service serv;

  queue<Point> q("somewhere", 8, ip::read_write);


  for (unsigned i=0; i<24; ++i)
    {
      message<Point> msg = q.create();
      asio::deadline_timer timer(serv, posix_time::seconds(1));
      timer.wait();
    }

}


