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
#include <ecto/util.hpp>

using boost::asio::ip::tcp;
namespace pt = boost::posix_time;
namespace asio = boost::asio;

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

  typedef boost::shared_ptr<tcp_connection> pointer;

  static pointer create(asio::io_service& io_service, unsigned blocksize_)
  {
    std::cout << "srv created with blocksize=" << blocksize_ << "\n";
    pointer p(new tcp_connection(io_service));
    p->nblocks = 0;
    p->blocksize = blocksize_;
    
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
    unsigned n_ints = blocksize / 4;
    //std::cout << "that is, " << n_ints << " unsigned ints.\n";
    const std::vector<uint32_t>& msg = gen_data(n_ints);

    asio::async_write(socket_, asio::buffer(msg),
		      boost::bind(&tcp_connection::handle_write, 
				  shared_from_this(),
				  asio::placeholders::error,
				  asio::placeholders::bytes_transferred));

    perfreport_timer.expires_from_now(pt::seconds(30));
    perfreport_timer.async_wait(boost::bind(&tcp_connection::handle_perfreport,
					    shared_from_this()));

  }

private:
  tcp_connection(asio::io_service& io_service)
    : socket_(io_service)
    , serv_(io_service)
    , perfreport_timer(io_service)
  {
  }

  void handle_perfreport()
  {
    std::cout << __PRETTY_FUNCTION__ << "\n";
    //    perfreport_timer.expires_from_now(pt::seconds(30));
    //    //    perfreport_timer.async_wait(boost::bind(&tcp_connection::handle_perfreport,
    //					    shared_from_this(),
    //					    asio::placeholders::error));
    
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

    if (nblocks == 100000)
      {
	pt::time_duration elapsed = pt::microsec_clock::local_time() - start_time;

	float meg_per_second = ((1000.0 * nblocks * blocksize) 
				/ (1024*1024)) / elapsed.total_milliseconds();
	std::cout << elapsed.total_milliseconds()/1000.0 << "s "
		  << meg_per_second << " meg/second\n";
	init();
      }
  }

  tcp::socket socket_;
  asio::io_service& serv_;
  asio::deadline_timer perfreport_timer;
};

class tcp_server
{
public:
  tcp_server(asio::io_service& io_service, unsigned blocksize)
    : acceptor_(io_service, tcp::endpoint(tcp::v4(), 10101))
    , blocksize_(blocksize)
      
  {
    start_accept();
  }

private:
  void start_accept()
  {
    tcp_connection::pointer new_connection =
      tcp_connection::create(acceptor_.io_service(), blocksize_);

    acceptor_.async_accept(new_connection->socket(),
        boost::bind(&tcp_server::handle_accept, this, new_connection,
          asio::placeholders::error));
  }

  void handle_accept(tcp_connection::pointer new_connection,
      const boost::system::error_code& error)
  {
    if (!error)
    {
      std::cout << "accepted.   blocksize=" << blocksize_ << "\n";
      new_connection->init();
      new_connection->start();
      start_accept();
    }
  }

  tcp::acceptor acceptor_;
  unsigned blocksize_;
};

int main(int argc, char** argv)
{
  try
  {
    asio::io_service io_service;
    tcp_server server(io_service, atoi(argv[1]));
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
