#include "websocket_server.hpp"

#include <format>
#include <iostream>

#include <boost/beast/websocket.hpp>

#include <ros/console.h>

using namespace std::chrono_literals;

WebsocketServer::WebsocketServer(std::string_view host, std::uint16_t port, handler_t&& on_open, handler_t&& on_close)
    : m_acceptor{m_context}, m_on_open{std::move(on_open)}, m_on_close{std::move(on_close)} {
    std::scoped_lock guard{m_mutex};

    tcp::endpoint endpoint{net::ip::make_address(host), port};

    ROS_INFO_STREAM("Starting streaming server @" << endpoint);

    m_acceptor.open(endpoint.protocol());
    m_acceptor.set_option(net::socket_base::reuse_address{true});
    m_acceptor.bind(endpoint);
    m_acceptor.listen(net::socket_base::max_listen_connections);

    acceptAsync();

    m_io_thread = std::jthread{[this] {
        ROS_INFO("IO service started");
        m_context.run();
        std::cout << "IO service stopped" << std::endl;
    }};
}

auto WebsocketServer::acceptAsync() -> void {
    m_acceptor.async_accept(m_context, [this](beast::error_code const& ec, tcp::socket socket) {
        std::scoped_lock guard{m_mutex};

        if (ec) {
            ROS_WARN_STREAM(std::format("Failed to accept: {}", ec.message()));
            return;
        }

        if (m_client) {
            try {
                m_client->close(websocket::close_code::normal);
            } catch (std::exception const& e) {
                ROS_WARN_STREAM(std::format("Exception closing existing client: {}", e.what()));
            }
            m_client.reset();

            if (m_on_close) m_on_close();
        }

        m_client.emplace(std::move(socket));

        m_client->binary(true);
        m_client->set_option(websocket::stream_base::timeout{
                .handshake_timeout = 3s,
                .idle_timeout = 3s,
                .keep_alive_pings = true,
        });
        m_client->set_option(websocket::stream_base::decorator([](websocket::response_type& res) {
            res.set(http::field::server, std::format("{} {}", BOOST_BEAST_VERSION_STRING, "mrover-stream-server"));
        }));

        m_client->accept();

        // For some DUMB REASON we have to read something so that we properly handle when the other side closes the connection
        m_client->async_read_some(boost::asio::mutable_buffer(nullptr, 0), [](beast::error_code const&, std::size_t) {});

        if (m_on_open) m_on_open();

        acceptAsync();
    });
}

auto WebsocketServer::feed(std::span<std::byte> data) -> void {
    net::mutable_buffer buffer{data.data(), data.size()};
    try {
        // TODO(quintin): async write?
        m_client->write(buffer);
    } catch (std::exception const& e) {
        ROS_WARN_STREAM(std::format("Exception writing to client: {}", e.what()));
        m_client.reset();

        if (m_on_close) m_on_close();
    }
}

auto WebsocketServer::isConnected() -> bool {
    std::scoped_lock guard{m_mutex};
    return m_client.has_value();
}

WebsocketServer::~WebsocketServer() {
    std::scoped_lock guard{m_mutex};
    try {
        m_client->close(websocket::close_code::normal);
    } catch (std::exception const&) {
    }
    m_acceptor.close();
    m_context.stop();
}
