//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2010 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using boost::asio::ip::tcp;
namespace pt = boost::posix_time;

std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

class tcp_connection
  : public boost::enable_shared_from_this<tcp_connection>
{
public:
  
  pt::ptime start_time;
  unsigned nblocks, blocksize;
  std::string the_message;

  typedef boost::shared_ptr<tcp_connection> pointer;

  static pointer create(boost::asio::io_service& io_service, unsigned blocksize_)
  {
    pointer p(new tcp_connection(io_service));
    p->nblocks = 0;
    p->blocksize = blocksize_;
    for (int i=0; i<blocksize_; ++i)
      {
	p->the_message += ('A' + (i % 16));
      }
    
    return p;
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  void init() {
    start_time = pt::microsec_clock::local_time();
    nblocks = 0;
  }
  void start()
  {
     boost::asio::async_write(socket_, boost::asio::buffer(the_message),
        boost::bind(&tcp_connection::handle_write, shared_from_this(),
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

private:
  tcp_connection(boost::asio::io_service& io_service)
    : socket_(io_service)
  {
  }

  void handle_write(const boost::system::error_code& /*error*/,
      size_t bytes_transferred)
  {
    ++nblocks;
    if (bytes_transferred != blocksize)
      {
	std::cout << "only " << bytes_transferred << " transferred, ending thread.\n";
	return;
      }
    start();

    if (nblocks > 1000000)
      {
	pt::time_duration elapsed = pt::microsec_clock::local_time() - start_time;

	float meg_per_second = ((1000.0 * nblocks * blocksize) 
				/ (1024*1024)) / elapsed.total_milliseconds();
	std::cout << nblocks << " " << elapsed.total_milliseconds() << " "
		  << meg_per_second << " meg/second\n";
	init();
      } 
  }

  tcp::socket socket_;
  std::string message_;
};

class tcp_server
{
public:
  tcp_server(boost::asio::io_service& io_service)
    : acceptor_(io_service, tcp::endpoint(tcp::v4(), 10101))
  {
    start_accept();
  }

private:
  void start_accept()
  {
    tcp_connection::pointer new_connection =
      tcp_connection::create(acceptor_.io_service(), 1024);

    acceptor_.async_accept(new_connection->socket(),
        boost::bind(&tcp_server::handle_accept, this, new_connection,
          boost::asio::placeholders::error));
  }

  void handle_accept(tcp_connection::pointer new_connection,
      const boost::system::error_code& error)
  {
    if (!error)
    {
      new_connection->init();
      new_connection->start();
      start_accept();
    }
  }

  tcp::acceptor acceptor_;
};

int main()
{
  try
  {
    boost::asio::io_service io_service;
    tcp_server server(io_service);
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
