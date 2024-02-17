#include "streaming.hpp"

#include <format>
#include <iostream>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/websocket.hpp>

StreamServer::StreamServer(std::string_view host, std::uint16_t port)
    : m_acceptor{m_context} {

    tcp::socket socket{m_context};
    beast::error_code ec;
    tcp::endpoint endpoint{net::ip::make_address(host), port};

    std::cout << "Starting H.265 streaming server @" << endpoint << std::endl;

    m_acceptor.open(endpoint.protocol(), ec);
    if (ec) throw std::runtime_error{std::format("Failed to open acceptor: {}", ec.message())};

    m_acceptor.set_option(net::socket_base::reuse_address(true), ec);
    if (ec) throw std::runtime_error{std::format("Failed to set socket option: {}", ec.message())};

    m_acceptor.bind(endpoint, ec);
    if (ec) throw std::runtime_error{std::format("Failed to bind: {}", ec.message())};

    m_acceptor.listen(net::socket_base::max_listen_connections, ec);
    if (ec) throw std::runtime_error{std::format("Failed to listen: {}", ec.message())};

    m_client.emplace(m_acceptor.accept());

    m_client->binary(true);
    m_client->set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    m_client->set_option(websocket::stream_base::decorator([](websocket::response_type& res) {
        res.set(http::field::server, std::format("{} {}", BOOST_BEAST_VERSION_STRING, "mrover-stream-server"));
    }));
    m_client->accept();

    std::cout << "Client connected" << std::endl;
}

void StreamServer::feed(std::span<std::byte> data) {
    net::mutable_buffer buffer{data.data(), data.size()};
    m_client->write(buffer);
}
