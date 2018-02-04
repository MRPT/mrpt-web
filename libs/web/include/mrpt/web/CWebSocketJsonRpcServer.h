#pragma once

#include <jsonrpccpp/server/abstractserverconnector.h>
#include <memory>
#include <boost/beast/websocket.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/bind_executor.hpp>
#include <thread>


using tcp = boost::asio::ip::tcp;               // from <boost/asio/ip/tcp.hpp>
namespace http = boost::beast::http;            // from <boost/beast/http.hpp>
namespace websocket = boost::beast::websocket;


// Report a failure
void
fail(boost::system::error_code ec, char const* what)
{
    std::cerr << (std::string(what) + ": " + ec.message() + "\n");
}

// Adjust settings on the stream
template<class NextLayer>
void
setup_stream(websocket::stream<NextLayer>& ws)
{
    // These values are tuned for Autobahn|Testsuite, and
    // should also be generally helpful for increased performance.

    websocket::permessage_deflate pmd;
    pmd.client_enable = true;
    pmd.server_enable = true;
    pmd.compLevel = 3;
    ws.set_option(pmd);

    ws.auto_fragment(false);

    // Autobahn|Testsuite needs this
    ws.read_message_max(64 * 1024 * 1024);
}

// Echoes back all received WebSocket messages
class async_session : public std::enable_shared_from_this<async_session>
{
    websocket::stream<tcp::socket> ws_;
    boost::asio::strand<
        boost::asio::io_context::executor_type> strand_;
    boost::beast::multi_buffer buffer_;
    std::function<std::string(const std::string &str)>
      m_request;
public:
    // Take ownership of the socket
    explicit
    async_session(tcp::socket socket,
      std::function<std::string(const std::string &str)> request
    )
        : ws_(std::move(socket))
        , strand_(ws_.get_executor())
        , m_request(request)
    {
        setup_stream(ws_);
    }

    // Start the asynchronous operation
    void
    run()
    {
        // Accept the websocket handshake
        ws_.async_accept_ex(
            [](websocket::response_type& res)
            {
                res.set(http::field::server,
                    "Boost.Beast/" + std::to_string(BOOST_BEAST_VERSION) + "-Async");
            },
            boost::asio::bind_executor(
                strand_,
                std::bind(
                    &async_session::on_accept,
                    shared_from_this(),
                    std::placeholders::_1)));
    }

    void
    on_accept(boost::system::error_code ec)
    {
        if(ec)
            return fail(ec, "accept");

        // Read a message
        do_read();
    }

    void
    do_read()
    {
        // Read a message into our buffer
        ws_.async_read(
            buffer_,
            boost::asio::bind_executor(
                strand_,
                std::bind(
                    &async_session::on_read,
                    shared_from_this(),
                    std::placeholders::_1,
                    std::placeholders::_2)));
    }

    void
    on_read(
        boost::system::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        // This indicates that the async_session was closed
        if(ec == websocket::error::closed)
            return;

        if(ec)
            fail(ec, "read");

        // Echo the message
        ws_.text(ws_.got_text());
        ws_.async_write(
            buffer_.data(),
            boost::asio::bind_executor(
                strand_,
                std::bind(
                    &async_session::on_write,
                    shared_from_this(),
                    std::placeholders::_1,
                    std::placeholders::_2)));
    }

    void
    on_write(
        boost::system::error_code ec,
        std::size_t bytes_transferred)
    {
        boost::ignore_unused(bytes_transferred);

        if(ec)
            return fail(ec, "write");

        // Clear the buffer
        buffer_.consume(buffer_.size());

        // Do another read
        do_read();
    }
};


// Accepts incoming connections and launches the sessions
class async_listener : public std::enable_shared_from_this<async_listener>
{
    boost::asio::strand<
        boost::asio::io_context::executor_type> strand_;
    tcp::acceptor acceptor_;
    tcp::socket socket_;
    std::function<std::string(const std::string &str)>
      m_request;
public:
    async_listener(
        boost::asio::io_context& ioc,
        std::function<std::string(const std::string &str)>
        request,
        tcp::endpoint endpoint
      )
        : strand_(ioc.get_executor())
        , acceptor_(ioc)
        , socket_(ioc)
        , m_request(request)
    {
        boost::system::error_code ec;

        // Open the acceptor
        acceptor_.open(endpoint.protocol(), ec);
        if(ec)
        {
            fail(ec, "open");
            return;
        }

        // Bind to the server address
        acceptor_.bind(endpoint, ec);
        if(ec)
        {
            fail(ec, "bind");
            return;
        }

        // Start listening for connections
        acceptor_.listen(
            boost::asio::socket_base::max_listen_connections, ec);
        if(ec)
        {
            fail(ec, "listen");
            return;
        }
    }

    // Start accepting incoming connections
    void
    run()
    {
        if(! acceptor_.is_open())
            return;
        do_accept();
    }

    void
    do_accept()
    {
        acceptor_.async_accept(
            socket_,
            boost::asio::bind_executor(
                strand_,
                std::bind(
                    &async_listener::on_accept,
                    shared_from_this(),
                    std::placeholders::_1)));
    }

    void
    on_accept(boost::system::error_code ec)
    {
        if(ec)
        {
            fail(ec, "accept");
        }
        else
        {
            // Create the async_session and run it
            std::make_shared<async_session>(std::move(socket_), m_request)->run();
        }

        // Accept another connection
        do_accept();
    }
};

class CWebSocketJsonRpcServer : public jsonrpc::AbstractServerConnector {
public:
  template <typename T>
  CWebSocketJsonRpcServer(T address, uint16_t port) :
  m_endpoint({
    boost::asio::ip::make_address(address),
    port})
  {
  }

  ~CWebSocketJsonRpcServer()
  {
      if(m_thread.joinable())
      {
        ioc.stop();
        m_thread.join();
      }
  }
  /**
   * This method launches the listening loop that will handle client connections.
   * @return true for success, false otherwise.
   */
  bool StartListening() override
  {
    std::make_shared<async_listener>(
        ioc,
        [](const std::string & str)
        {
          return str;
        },
        m_endpoint
    )->run();
    m_thread = std::thread([&ioc = this->ioc]
        {
            ioc.run();
        });
  }

  /**
   * This method stops the listening loop that will handle client connections.
   * @return True if successful, false otherwise or if not listening.
   */
  bool StopListening() override
  {
    ioc.stop();
    m_thread.join();
  }

private:
  boost::asio::io_context ioc;
  std::thread m_thread;
  tcp::endpoint m_endpoint;
};
