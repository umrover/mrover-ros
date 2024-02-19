#include "streaming.hpp"

#include <format>
#include <iostream>

#include <boost/beast/websocket.hpp>

#include <ros/console.h>

using namespace std::chrono_literals;

StreamServer::StreamServer(std::string_view host, std::uint16_t port)
    : m_acceptor{m_context} {

    beast::error_code ec;
    tcp::endpoint endpoint{net::ip::make_address(host), port};

    ROS_INFO_STREAM("Starting H.265 streaming server @" << endpoint);

    m_acceptor.open(endpoint.protocol(), ec);
    if (ec) throw std::runtime_error{std::format("Failed to open acceptor: {}", ec.message())};

    m_acceptor.set_option(net::socket_base::reuse_address(true), ec);
    if (ec) throw std::runtime_error{std::format("Failed to set socket option: {}", ec.message())};

    m_acceptor.bind(endpoint, ec);
    if (ec) throw std::runtime_error{std::format("Failed to bind: {}", ec.message())};

    m_acceptor.listen(net::socket_base::max_listen_connections, ec);
    if (ec) throw std::runtime_error{std::format("Failed to listen: {}", ec.message())};
}

auto StreamServer::feed(std::span<std::byte> data) -> bool {
    if (!m_client) {
        m_client.emplace(m_acceptor.accept());
        m_client->binary(true);
        m_client->set_option(websocket::stream_base::timeout{
                .handshake_timeout = 3s,
                .idle_timeout = 3s,
                .keep_alive_pings = true,
        });
        m_client->set_option(websocket::stream_base::decorator([](websocket::response_type& res) {
            res.set(http::field::server, std::format("{} {}", BOOST_BEAST_VERSION_STRING, "mrover-stream-server"));
        }));
        ROS_INFO("Waiting for client..");
        m_client->accept();
        ROS_INFO("Client connected");
    }

    net::mutable_buffer buffer{data.data(), data.size()};
    try {
        m_client->write(buffer);
        return true;
    } catch (std::exception const& e) {
        ROS_WARN_STREAM(std::format("Exception writing to client: {}", e.what()));
        m_client.reset();
        return false;
    }
}
