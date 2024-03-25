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
    ROS_INFO_STREAM("Starting websocket server @" << endpoint);

    m_acceptor.open(endpoint.protocol());
    m_acceptor.set_option(net::socket_base::reuse_address{true});
    m_acceptor.bind(endpoint);
    m_acceptor.listen(4);

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

        websocket_t& client = m_clients.emplace_back(std::move(socket));

        client.binary(true);
        client.set_option(websocket::stream_base::timeout{
                .handshake_timeout = 3s,
                .idle_timeout = 3s,
                .keep_alive_pings = true,
        });
        client.set_option(websocket::stream_base::decorator([](websocket::response_type& res) {
            res.set(http::field::server, std::format("{} {}", BOOST_BEAST_VERSION_STRING, "mrover-websocket-server"));
        }));

        client.accept();

        // For some DUMB REASON we have to read something so that we properly handle when the other side closes the connection
        // See: https://live.boost.org/doc/libs/1_84_0/libs/beast/doc/html/beast/using_websocket/control_frames.html
        client.async_read_some(boost::asio::mutable_buffer(nullptr, 0), [](beast::error_code const&, std::size_t) {});

        if (m_on_open) m_on_open();

        acceptAsync();
    });
}

auto WebsocketServer::broadcast(std::span<std::byte> data) -> void {
    std::scoped_lock guard{m_mutex};
    net::const_buffer buffer{data.data(), data.size()};
    for (auto it = m_clients.begin(); it != m_clients.end();) {
        try {
            // TODO(quintin): Could async write here, but need to garuntee that the buffer stays alive until the write is done
            it->write(buffer);
            ++it;
        } catch (std::exception const& e) {
            ROS_WARN_STREAM(std::format("Exception writing to client: {}", e.what()));
            it = m_clients.erase(it);

            if (m_on_close) m_on_close();
        }
    }
}

auto WebsocketServer::clientCount() -> std::size_t {
    std::scoped_lock guard{m_mutex};
    return m_clients.size();
}

WebsocketServer::~WebsocketServer() {
    std::scoped_lock guard{m_mutex};
    for (websocket_t& client: m_clients) {
        try {
            client.close(websocket::close_code::normal);
        } catch (std::exception const&) {
        }
    }
    m_acceptor.close();
    m_context.stop();
}
