/**
 * CZoomyServer.h - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#pragma once

#include <iostream>
#include <CUDPServer.hpp>
#include "CCommonBase.hpp"

class CZoomyServer : public CCommonBase {
private:
    // net
    std::string _port;
    sockaddr_in _client{};
    CUDPServer _server;
    std::thread _thread_tx, _thread_rx;
    std::chrono::steady_clock::time_point _timeout_count;
    int _time_since_start;
    std::queue<std::string> _tx_queue, _rx_queue;

    std::vector<uint8_t> _rx_buf;
    long _rx_bytes;

    void rx();
    void tx();

public:
    CZoomyServer(std::string port);
    ~CZoomyServer();

    void update() override;
    void draw() override;

    static void thread_rx(CZoomyServer *who_called);
    static void thread_tx(CZoomyServer *who_called);
};
