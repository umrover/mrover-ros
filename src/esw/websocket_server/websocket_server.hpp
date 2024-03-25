#pragma once

#include <cstdint>
#include <list>
#include <span>
#include <string_view>
#include <thread>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/websocket.hpp>

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;

using tcp = net::ip::tcp;

using websocket_t = websocket::stream<beast::tcp_stream>;

class WebsocketServer {

    using handler_t = std::function<void()>;

    net::io_context m_context;
    tcp::acceptor m_acceptor;
    std::list<websocket_t> m_clients; // Linked lists since we need to remove while iterating in the feed function
    std::jthread m_io_thread;
    handler_t m_on_open, m_on_close;
    std::recursive_mutex m_mutex; // User supplied callbacks could call our public functions, avoid self deadlock

public:
    WebsocketServer(std::string_view host, std::uint16_t port, handler_t&& on_open, handler_t&& on_close);

    ~WebsocketServer();

    auto acceptAsync() -> void;

    auto broadcast(std::span<std::byte> data) -> void;

    auto clientCount() -> std::size_t;
};
