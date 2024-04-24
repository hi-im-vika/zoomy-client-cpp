/**
 * CZoomyServer.cpp - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CZoomyServer.hpp"

#define PING_TIMEOUT 1000
#define NET_DELAY 1

CZoomyServer::CZoomyServer(std::string port) {
    _port = port;

    // net init
    _timeout_count = std::chrono::steady_clock::now();
    _time_since_start = 0;

    _server.setup(_port);

    // start listen thread
    _thread_rx = std::thread(thread_rx, this);
    _thread_rx.detach();

    // start send thread
    _thread_tx = std::thread(thread_tx, this);
    _thread_tx.detach();
}

CZoomyServer::~CZoomyServer() = default;

void CZoomyServer::update() {
//    spdlog::info("Server update");

    for (; !_rx_queue.empty(); _rx_queue.pop()) {

//            // acknowledge next data in queue
//            spdlog::info("New in RX queue: " + rx_queue.front());
//            spdlog::info("Remaining in queue: " + std::to_string(rx_queue.size()));

        // echo client data back to client
        _tx_queue.emplace(_rx_queue.front());
    }

    _time_since_start = (int) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _timeout_count).count();
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::draw() {
//    spdlog::info("Server draw");
    // servo control code goes here...
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(10));
}

void CZoomyServer::rx() {
    _rx_bytes = 0;
    _rx_buf.clear();
    _server.do_rx(_rx_buf, _client, _rx_bytes);
    std::string temp = std::string(_rx_buf.begin(),_rx_buf.begin() + _rx_bytes);
    // only add to rx queue if data is not empty
    if(!temp.empty()) _rx_queue.emplace(temp);
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::tx() {
    for (; !_tx_queue.empty(); _tx_queue.pop()) {
//            spdlog::info("Sending");
        std::vector<uint8_t> tx_buf(_tx_queue.front().begin(),_tx_queue.front().end());
        _server.do_tx(tx_buf,_client);
    }
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::thread_rx(CZoomyServer *who_called) {
    while (!who_called->_do_exit) {
        who_called->rx();
    }
}

void CZoomyServer::thread_tx(CZoomyServer *who_called) {
    while (!who_called->_do_exit) {
        who_called->tx();
    }
}

int main() {
//    std::cout << "Hello, World!" << std::endl;
    CZoomyServer c = CZoomyServer("46188");
    c.run();
    return 0;
}