#ifndef UDP_SERVER_H_
#define UDP_SERVER_H_

#include <memory>
#include <mutex>
#include <sstream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>

using boost::asio::deadline_timer;
using boost::asio::ip::udp;

class UDPServerRealTime
{
public:
  UDPServerRealTime(const udp::endpoint& local_endpoint) : socket_(io_service_, local_endpoint), deadline_(io_service_)
  {
    // set the deadline to positive infinity so that the actor takes no action
    // until a specific deadline is set.
    deadline_.expires_at(boost::posix_time::pos_infin);
    // Start the persistent actor that checks for deadline expiry.
    checkDeadline();
  }

  bool waitForConnection()
  {
    std::cout << "Waiting for connection from client" << std::endl;
    udp::endpoint remote_endpoint;
    size_t length = socket_.receive_from(boost::asio::buffer(buffer_), remote_endpoint);
    remote_endpoint_ = remote_endpoint;
    std::cout << "Connected to: " << remote_endpoint_ << std::endl;
    return true;
  }

  bool waitForConnection(const std::string code)
  {
    std::cout << "Waiting for connection from client" << std::endl;
    udp::endpoint remote_endpoint;
    size_t length = socket_.receive_from(boost::asio::buffer(buffer_), remote_endpoint);
    if (std::string(buffer_, buffer_ + length) == code)
    {
      remote_endpoint_ = remote_endpoint;
      std::cout << "Connected to: " << remote_endpoint_ << std::endl;
      return true;
    }
    else
    {
      std::cout << "Erronous connection attempt from: " << remote_endpoint << std::endl;
      return false;
    }
  }

  std::size_t send(const boost::asio::mutable_buffer& buffer)
  {
      return socket_.send_to(boost::asio::buffer(buffer), remote_endpoint_);
  }

  std::size_t receiveTimeout(const boost::asio::mutable_buffer& buffer, boost::posix_time::time_duration timeout)
  {
      timeout_flag_ = false;
      boost::system::error_code ec;
      std::size_t length = receive(buffer, timeout, ec);
      if (ec)
      {
          timeout_flag_ = true;
      }
      return length;
  }

  std::size_t receive(const boost::asio::mutable_buffer& buffer)
  {
      return socket_.receive(boost::asio::buffer(buffer));
  }

  std::size_t receive(const boost::asio::mutable_buffer& buffer,
          boost::posix_time::time_duration timeout, boost::system::error_code& ec)
  {
    // Set a deadline for the asynchronous operation.
    deadline_.expires_from_now(timeout);

    // Set up the variables that receive the result of the asynchronous
    // operation. The error code is set to would_block to signal that the
    // operation is incomplete. Asio guarantees that its asynchronous
    // operations will never fail with would_block, so any other value in
    // ec indicates completion.
    ec = boost::asio::error::would_block;
    std::size_t length = 0;

    // Start the asynchronous operation itself. The handleReceive function
    // used as a callback will update the ec and length variables.
    socket_.async_receive(boost::asio::buffer(buffer),
                          boost::bind(&UDPServerRealTime::handleReceive, _1, _2, &ec, &length));

    // Block until the asynchronous operation has completed.
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    return length;
  }

  std::string local_endpoint()
  {
    std::stringstream ss;
    ss << socket_.local_endpoint();
    return ss.str();
  }

  std::string remote_endpoint()
  {
    std::stringstream ss;
    ss << remote_endpoint_;
    return ss.str();
  }

  bool timeout()
  {
      return timeout_flag_;
  }

private:

  void checkDeadline()
  {
    // Check whether the deadline has passed. We compare the deadline against
    // the current time since a new asynchronous operation may have moved the
    // deadline before this actor had a chance to run.
    if (deadline_.expires_at() <= deadline_timer::traits_type::now())
    {
      // The deadline has passed. The outstanding asynchronous operation needs
      // to be cancelled so that the blocked receive() function will return.
      //
      // Please note that cancel() has portability issues on some versions of
      // Microsoft Windows, and it may be necessary to use close() instead.
      // Consult the documentation for cancel() for further information.
      socket_.cancel();

      // There is no longer an active deadline. The expiry is set to positive
      // infinity so that the actor takes no action until a new deadline is set.
      deadline_.expires_at(boost::posix_time::pos_infin);
    }

    // Put the actor back to sleep.
    deadline_.async_wait(boost::bind(&UDPServerRealTime::checkDeadline, this));
  }

  static void handleReceive(const boost::system::error_code& ec, std::size_t length,
                            boost::system::error_code* out_ec, std::size_t* out_length)
  {
    *out_ec = ec;
    *out_length = length;
  }

private:

  static const size_t BUFFER_SIZE = 1024;
  char buffer_[BUFFER_SIZE];

  //static const unsigned int timeout_count_;
  bool timeout_flag_;

  boost::asio::io_service io_service_;
  udp::socket socket_;
  udp::endpoint local_endpoint_;
  udp::endpoint remote_endpoint_;

  deadline_timer deadline_;
};

#endif
