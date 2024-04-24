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

    // OpenCV init

    /**
     * camera selection
     * facetime hd camera:
     * _video_capture.open(0);
     * continuity:
     * _video_capture.open(2);
     * raspberry pi:
     * _video_capture = cv::VideoCapture("libcamerasrc ! video/x-raw, width=128, height=96 ! appsink", cv::CAP_GSTREAMER);
     */

//    _video_capture = cv::VideoCapture("libcamerasrc ! video/x-raw, width=128, height=96 ! appsink", cv::CAP_GSTREAMER);
//    _video_capture.open(0);
//    _video_capture.open(2);
    _video_capture = cv::VideoCapture("autovideosrc ! video/x-raw, framerate=30/1 ! videoscale ! videoconvert ! appsink", cv::CAP_GSTREAMER);
    if (!_video_capture.isOpened()) {
        spdlog::error("Could not open camera.");
        exit(-1);
    } else {
        spdlog::info("Camera opened with backend " + _video_capture.getBackendName());
        _video_capture.read(_frame);
    }
}

CZoomyServer::~CZoomyServer() = default;

void CZoomyServer::update() {
    for (; !_rx_queue.empty(); _rx_queue.pop()) {
//        _video_capture.read(_frame);
        cv::Mat payload = cv::Mat::ones(cv::Size(128, 96), CV_8UC3);
        cv::randu(payload, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        std::vector<uint8_t> encoded;
        cv::imencode(".jpg", payload, encoded);
//        _video_capture.read(_frame);

//            // acknowledge next data in queue
//            spdlog::info("New in RX queue: " + rx_queue.front());
//            spdlog::info("Remaining in queue: " + std::to_string(rx_queue.size()));

        // echo client data back to client
//        _tx_queue.emplace(std::string(encoded.begin, encoded);
        std::vector<uint8_t> tx_assembled(_rx_queue.front());
        tx_assembled.insert(tx_assembled.end(),encoded.begin(),encoded.end());
        _tx_queue.emplace(tx_assembled);
    }

    _time_since_start = (int) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _timeout_count).count();
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::draw() {
    // servo control code goes here...
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(10));
}

void CZoomyServer::rx() {
    _rx_bytes = 0;
    _rx_buf.clear();
    _server.do_rx(_rx_buf, _client, _rx_bytes);
    std::vector<uint8_t> temp(_rx_buf.begin(),_rx_buf.begin() + _rx_bytes);
    // only add to rx queue if data is not empty
    if(!temp.empty()) _rx_queue.emplace(temp);
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::tx() {
    for (; !_tx_queue.empty(); _tx_queue.pop()) {
//            spdlog::info("Sending");
        _server.do_tx(_tx_queue.front(),_client);
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