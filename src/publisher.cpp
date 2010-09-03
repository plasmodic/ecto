#include <ros/util.hpp>
#include <ros/message.hpp>
#include <ros/queue.hpp>

#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost;

namespace ip = boost::interprocess;

struct Point {
  float x, y, z;
  friend std::ostream& operator<<(std::ostream& os, const Point& p) {
    return os << "[Point @" << &p << " " 
	      << p.x << " " << p.y << " " << p.z << "]";
  }
};

int main(int argc, char** argv)
{

  asio::io_service serv;

  queue<Point> q("somewhere", 8, ip::read_write);


  for (unsigned i=0; i<24; ++i)
    {
      queue<Point>::ptr msg = q.create();
      Point& p = *msg;
      p.x = i * 1.01;
      p.y = i * 1.02;
      std::cout << q;
      std::cout << p << std::endl;
      asio::deadline_timer timer(serv, posix_time::seconds(1));
      timer.wait();
    }

}


