#pragma once

#include <smacc/impl/smacc_state_impl.h>
#include <smacc/smacc_client.h>

#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/optional/optional_io.hpp>
#include <functional>
#include <thread>

namespace smacc {
namespace client_bases {

class session : public std::enable_shared_from_this<session> {
  boost::asio::ip::tcp::resolver resolver_;
  boost::beast::tcp_stream stream_;
  boost::beast::flat_buffer buffer_;  // (Must persist between reads)
  boost::beast::http::request<boost::beast::http::empty_body> req_;
  boost::beast::http::response<boost::beast::http::string_body> res_;

 public:
  // Objects are constructed with a strand to
  // ensure that handlers do not execute concurrently.
  session(boost::asio::io_context &ioc,
          const std::function<void(const boost::beast::http::response<
                                   boost::beast::http::string_body> &)>
              response)
      : resolver_(boost::asio::make_strand(ioc)),
        stream_(boost::asio::make_strand(ioc)),
        onResponse{response} {}

  // Start the asynchronous operation
  void run(const std::string &host, const std::string &port,
           const std::string &target, const int &version) {
    // Set up an HTTP GET request message
    req_.version(version);
    req_.method(boost::beast::http::verb::get);
    req_.target(target);
    req_.set(boost::beast::http::field::host, host);
    req_.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);

    // Look up the domain name
    resolver_.async_resolve(host.c_str(), port.c_str(),
                            boost::beast::bind_front_handler(
                                &session::on_resolve, shared_from_this()));
  }

  void on_resolve(boost::beast::error_code ec,
                  boost::asio::ip::tcp::resolver::results_type results) {
    if (ec) return fail(ec, "resolve");

    // Set a timeout on the operation
    stream_.expires_after(std::chrono::seconds(30));

    // Make the connection on the IP address we get from a lookup
    stream_.async_connect(results,
                          boost::beast::bind_front_handler(&session::on_connect,
                                                           shared_from_this()));
  }

 private:
  void fail(boost::beast::error_code ec, char const *what) {
    std::cerr << what << ": " << ec.message() << "\n";
    res_.result(boost::beast::http::status::bad_request);
    res_.reason() = ec.message();
    onResponse(res_);
  }

  void on_connect(boost::beast::error_code ec,
                  boost::asio::ip::tcp::resolver::results_type::endpoint_type) {
    if (ec) return fail(ec, "connect");

    // Set a timeout on the operation
    stream_.expires_after(std::chrono::seconds(30));

    // Send the HTTP request to the remote host
    boost::beast::http::async_write(
        stream_, req_,
        boost::beast::bind_front_handler(&session::on_write,
                                         shared_from_this()));
  }

  void on_write(boost::beast::error_code ec, std::size_t bytes_transferred) {
    boost::ignore_unused(bytes_transferred);

    if (ec) return fail(ec, "write");

    // Receive the HTTP response
    boost::beast::http::async_read(stream_, buffer_, res_,
                                   boost::beast::bind_front_handler(
                                       &session::on_read, shared_from_this()));
  }

  void on_read(boost::beast::error_code ec, std::size_t bytes_transferred) {
    boost::ignore_unused(bytes_transferred);

    if (ec) return fail(ec, "read");

    // Gracefully close the socket
    stream_.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);

    // not_connected happens sometimes so don't bother reporting it.
    if (ec && ec != boost::beast::errc::not_connected)
      return fail(ec, "shutdown");

    // If we get here then the connection is closed gracefully
    onResponse(res_);
  }

  std::function<void(const boost::beast::http::response<
                     boost::beast::http::string_body> &response)>
      onResponse;
};

class SmaccHttpClient : public smacc::ISmaccClient {
 public:
  boost::optional<std::string> serverName;
  boost::optional<int> timeout;

  SmaccHttpClient()
      : worker_guard_{boost::asio::make_work_guard(io_context_)},
        initialized_{false} {}

  SmaccHttpClient(const std::string &serverName, const int &timeout)
      : worker_guard_{boost::asio::make_work_guard(io_context_)},
        initialized_{false} {
    this->serverName = serverName;
    this->timeout = timeout;
  }

  ~SmaccHttpClient() {
    worker_guard_.reset();
    tcp_connection_runner_.join();
  }

  template <typename T>
  boost::signals2::connection onResponseReceived(
      void (T::*callback)(const boost::beast::http::response<
                          boost::beast::http::string_body> &),
      T *object) {
    return this->getStateMachine()->createSignalConnection(onResponseReceived_,
                                                           callback, object);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation() {}

  virtual void initialize() {
    if (!initialized_) {
      if (!timeout) timeout = 2000;  // 2s timeout default
      if (!serverName) {
        ROS_ERROR("Server URL not set, skipping initialisation");
      } else {
        ROS_INFO_STREAM("[" << this->getName()
                            << "] Initialising HTTP client for " << serverName);
        tcp_connection_runner_ = std::thread{[&]() { io_context_.run(); }};
        initialized_ = true;
      }
    }
  }

  void makeRequest(const std::string &path) {
    std::make_shared<session>(io_context_, runSignal)
        ->run(serverName.get(), "80", path, 11);
  }

 private:
  smacc::SmaccSignal<void(
      const boost::beast::http::response<boost::beast::http::string_body> &)>
      onResponseReceived_;

  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<decltype(io_context_)::executor_type>
      worker_guard_;
  std::thread tcp_connection_runner_;

  bool initialized_;

  std::function<void(const boost::beast::http::response<
                     boost::beast::http::string_body> &response)>
      runSignal{[&](const boost::beast::http::response<
                    boost::beast::http::string_body> &response) {
        onResponseReceived_(response);
      }};
};
}  // namespace client_bases
}  // namespace smacc
