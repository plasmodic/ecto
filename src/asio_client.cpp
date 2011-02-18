//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2010 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <stdint.h>

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using boost::asio::ip::tcp;
namespace pt = boost::posix_time;

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage: client <host> <blocksize>" << std::endl;
      return 1;
    }


    boost::asio::io_service io_service;

    tcp::resolver resolver(io_service);
    tcp::resolver::query query(argv[1], "10101");
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    tcp::resolver::iterator end;

    tcp::socket socket(io_service);
    boost::system::error_code error = boost::asio::error::host_not_found;
    while (error && endpoint_iterator != end)
    {
      socket.close();
      socket.connect(*endpoint_iterator++, error);
    }
    if (error)
      throw boost::system::system_error(error);

    pt::ptime start_time = pt::microsec_clock::local_time();

    unsigned nblocks = 0;
    unsigned blocksize = atoi(argv[2]);
    std::vector<uint32_t> buf(blocksize/4);

    buf.reserve(blocksize);

    for (;;)
    {
      nblocks++;
      boost::system::error_code error;

      size_t len = socket.read_some(boost::asio::buffer(buf), error);

      if (error == boost::asio::error::eof)
        break; // Connection closed cleanly by peer.
      else if (error)
        throw boost::system::system_error(error); // Some other error.

      // std::cout.write(buf.data(), len);
      if (nblocks == 1000000)
	{
	  pt::time_duration elapsed = pt::microsec_clock::local_time() - start_time;

	  std::cout << ((1000.0 * nblocks * blocksize) 
			/ (1024*1024)) 
	    / elapsed.total_milliseconds()
		    << " megs/sec\n";
	  std::cout.flush();
	  start_time = pt::microsec_clock::local_time();
	  nblocks = 0;
	}
      
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "asio throw: " << e.what() << std::endl;
  }

  return 0;
}
