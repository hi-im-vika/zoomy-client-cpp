/**
 * CZoomyClient.h - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

#include <nlohmann/json.hpp>
#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_opengl3.h>
#include <spdlog/spdlog.h>
#include <SDL.h>

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
#include "CAutoController.hpp"

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
    GLuint _preview_tex;
    bool _use_dashcam;
    cv::Mat _dashcam_area, _arena_area;
    cv::Mat _dashcam_img, _dashcam_raw_img;
    cv::Mat _arena_img, _arena_raw_img;
    cv::Mat _arena_mask_img;
    cv::Mat _raw_mask;
    SDL_Event _evt;
    std::mutex _mutex_dashcam, _mutex_arena, _mutex_mask_gen;
    ImVec2 _arena_mouse_pos;
    int _wp_highlighted;
    char _host_udp[64];
    char _port_udp[64];
    char _host_tcp[64];
    char _port_tcp[64];
    int _cam_location;
    bool _use_local;

    // control
    std::ifstream _json_file;
    nlohmann::json _json_data;
    CAutoController _autonomous;
    unsigned int _step;
    std::vector<int> _values;
    std::vector<cv::Point> _joystick;
    SDL_GameController *_gc;
    bool _auto, _relation;
    std::string _xml_vals;
    std::vector <CAutoController::waypoint> _waypoints;

    // opencv
    cv::VideoCapture _video_capture;
    cv::VideoCapture _arena_capture;
    std::string _dashcam_gst_string;
    std::string _arena_gst_string;
    bool _flip_image;
    bool _show_mask;
    bool _show_waypoints;
    bool _show_preview;
    bool _use_auto;
    std::vector<std::string> _hsv_slider_names;
    cv::Scalar_<int> _hsv_threshold_low, _hsv_threshold_high;
    std::vector<int*> _pointer_hsv_thresholds;

    // opencv aruco
    std::vector<int> _marker_ids;
    std::vector<std::vector<cv::Point2f>> _marker_corners, _rejected_candidates;
    cv::aruco::DetectorParameters _detector_params;
    cv::aruco::Dictionary _dictionary;
    cv::aruco::ArucoDetector _detector;

    // opencv homography
    std::vector<cv::Point> _homography_corners;
    std::vector<ImVec2> _quad_points;
    std::vector<ImVec2> _quad_points_scaled;
    std::vector<double> _dist_quad_points;
    int _closest_quad_point;
    bool _show_homography;
    float _arena_scale_factor;
    float _coord_scale;
    ImVec2 _arena_last_cursor_pos;
    ImVec2 _last_car_pos;
    cv::Mat _arena_warped_img;

    // net (udp)
    bool _udp_req_ready;
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
    std::string _tcp_host;
    std::string _tcp_port;
    CTCPClient _tcp_client;
    std::thread _thread_update_tcp, _thread_tcp_tx, _thread_tcp_rx;
    std::queue<std::vector<uint8_t>> _tcp_tx_queue, _tcp_rx_queue;
    std::vector<uint8_t> _tcp_rx_buf;
    long _tcp_rx_bytes;
    bool _tcp_send_data;

    // draw specific UI elements
    void imgui_draw_settings();
    void imgui_draw_waypoints();
    void imgui_draw_dashcam();
    void imgui_draw_arena();
    void imgui_draw_debug();

    static void fit_texture_to_window(cv::Mat &input_image, GLuint &output_texture);
    static void fit_texture_to_window(cv::Mat &input_image, GLuint &output_texture, float &scale, ImVec2 &cursor_screen_pos_before_image);

    static void mat_to_tex(cv::Mat &input, GLuint &output);

    void udp_rx();
    void udp_tx();

    void tcp_rx();
    void tcp_tx();

public:
    CZoomyClient(cv::Size s);
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
