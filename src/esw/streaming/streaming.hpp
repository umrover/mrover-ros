#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/websocket.hpp>

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;

using tcp = net::ip::tcp;

class StreamServer {

    net::io_context m_context;
    tcp::acceptor m_acceptor;
    std::optional<websocket::stream<beast::tcp_stream>> m_client;

public:
    StreamServer(std::string_view host, std::uint16_t port);

    auto feed(std::span<std::byte> data) -> bool;
};
