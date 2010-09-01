#include <ros/util.hpp>
#include <ros/message.hpp>
#include <ros/queue.hpp>

#include <boost/asio.hpp>
#include <boost/noncopyable.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost;

namespace ip = boost::interprocess;

int main(int argc, char** argv)
{
  ip::shared_memory_object::remove("somewhere");
}


