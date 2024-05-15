/**
 * CZoomyClient.h - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#pragma once

#include <iostream>
#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_opengl3.h>
#include <spdlog/spdlog.h>
#include <SDL.h>
#include <sstream>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif

#include <CUDPClient.hpp>
#include <CTCPClient.hpp>

#include "CWindow.hpp"
#include "CCommonBase.hpp"
#include "CDPIHandler.hpp"

enum value_type {
    GC_LEFTX,
    GC_LEFTY,
    GC_RIGHTX,
    GC_RIGHTY,
    GC_LTRIG,
    GC_RTRIG,
    GC_A,
    GC_B,
    GC_X,
    GC_Y,
};

class CZoomyClient : public CCommonBase {
private:
    // imgui
    std::unique_ptr<CWindow> _window;
    GLuint _dashcam_tex;
    GLuint _arena_tex;
    cv::Mat _dashcam_area, _arena_area;
    cv::Mat _dashcam_img, _dashcam_raw_img;
    cv::Mat _arena_img, _arena_raw_img;
    SDL_Event _evt;
    std::mutex _lockout_dashcam, _lockout_arena;

    // control
    std::vector<int> _values;
    SDL_GameController *_gc;
    bool _do_invert_steering;
    int _steering_trim;
    int _throttle_trim;
    std::string _xml_vals;

    // opencv
    cv::VideoCapture _video_capture;
    bool _flip_image;
    bool _gstreamer_checkbox;
    std::string _gstreamer_string;
    const std::string _default_gstreamer_string = "udpsrc port=5200 ! application/x-rtp, media=video, clock-rate=90000, payload=96 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink\", cv::CAP_GSTREAMER";

    // opencv aruco
    std::vector<int> _marker_ids;
    std::vector<std::vector<cv::Point2f>> _marker_corners, _rejected_candidates;
    cv::aruco::DetectorParameters _detector_params;
    cv::aruco::Dictionary _dictionary;
    cv::aruco::ArucoDetector _detector;

    // net (udp)
    bool _udp_req_ready;
    bool _udp_checkbox;
    std::string _udp_host;
    std::string _udp_port;
    CUDPClient _udp_client;
    std::thread _thread_update_udp, _thread_udp_tx, _thread_udp_rx;
    std::chrono::steady_clock::time_point _udp_timeout_count;
    int _udp_time_since_start;
    std::queue<std::vector<uint8_t>> _udp_tx_queue, _udp_rx_queue;
    std::vector<uint8_t> _udp_rx_buf;
    long _udp_rx_bytes;
    bool _udp_send_data;

    // net (tcp)
    bool _tcp_req_ready;
    bool _tcp_checkbox;
    std::string _tcp_host;
    std::string _tcp_port;
    CTCPClient _tcp_client;
    std::thread _thread_update_tcp, _thread_tcp_tx, _thread_tcp_rx;
    std::queue<std::vector<uint8_t>> _tcp_tx_queue, _tcp_rx_queue;
    std::vector<uint8_t> _tcp_rx_buf;
    long _tcp_rx_bytes;
    bool _tcp_send_data;
    std::chrono::steady_clock::time_point _tcp_last_frame;

    void mat_to_tex(cv::Mat &input, GLuint &output);
    int normalize_with_trim(int i, int trim);

    void udp_rx();
    void udp_tx();

    void tcp_rx();
    void tcp_tx();

public:
    CZoomyClient(cv::Size s, std::string host, std::string port);
    ~CZoomyClient();

    void update() override;
    void draw() override;

    void update_udp();
    void update_tcp();

    static void thread_update_udp(CZoomyClient *who_called);
    static void thread_udp_rx(CZoomyClient *who_called);
    static void thread_udp_tx(CZoomyClient *who_called);

    static void thread_update_tcp(CZoomyClient *who_called);
    static void thread_tcp_rx(CZoomyClient *who_called);
    static void thread_tcp_tx(CZoomyClient *who_called);
};
