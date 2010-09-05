//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2010 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

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

    time_t starttime;
    time(&starttime);
    unsigned nblocks = 0;
    unsigned blocksize = atoi(argv[2]);
    std::vector<char> buf(blocksize);

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
      if (nblocks > 1000000)
	{
	  time_t curtime;
	  time(&curtime);
	  std::cout << ((1.0 * nblocks * blocksize) / (1024*1024)) / (curtime - starttime)
		    << " megs/sec\n";
	  std::cout.flush();
	  starttime = curtime;
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
