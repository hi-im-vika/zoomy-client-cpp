/**
 * CZoomyClient.cpp - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CZoomyClient.hpp"

#define PING_TIMEOUT 1000
#define NET_DELAY 35
#define DEADZONE 4096
#define ARENA_DIM 600
#define DEMO_SPEED 0.3

// increase this value if malloc_error_break happens too often
#define TCP_DELAY 100
//#define TCP_DELAY 15 // only if over ssh forwarding

CZoomyClient::CZoomyClient(cv::Size s) {
    _window_size = s;
    _angle = 0;
    _deltaTime = std::chrono::steady_clock::now();
    _demo = true;

    // SDL init
    uint init_flags = SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER;

    if (SDL_Init(init_flags) != 0) {
        spdlog::error("Error during SDL init");
        exit(-1);
    }

    std::stringstream ss;

    // control init

    int joysticks = SDL_NumJoysticks();
    if (!joysticks) {
        spdlog::warn("No controllers detected.");
    } else {
        for (int i = 0; i < joysticks; i++) {
            if (SDL_IsGameController(i)) {
                _gc = SDL_GameControllerOpen(i);
                ss << "Game controller " << i << " opened.";
                spdlog::info(ss.str());
            }
        }
    }

    _values = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // dear imgui init
    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#elif defined(__APPLE__)
    // GL 3.2 Core + GLSL 150
    const char *glsl_version = "#version 150";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif

    _window = std::make_unique<CWindow>(WINDOW_NAME, _window_size.width, _window_size.height);
    if (_window == nullptr) {
        spdlog::error("Error creating window");
        exit(-1);
    }

    SDL_GL_MakeCurrent(_window->get_native_window(), _window->get_native_context());
    SDL_GL_SetSwapInterval(1); // Enable vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
    io.ConfigDockingTransparentPayload = true;

    // scale fonts for DPI
    const float font_scaling_factor = CDPIHandler::get_scale();
    const float font_size = 16.0F * font_scaling_factor;
    const std::string font_path = "../res/font/inter.ttf";

    io.Fonts->AddFontFromFileTTF(font_path.c_str(), font_size);
    io.FontDefault = io.Fonts->AddFontFromFileTTF(font_path.c_str(), font_size);
    CDPIHandler::set_global_font_scaling(&io);

    // rendering init

    ImGui_ImplSDL2_InitForOpenGL(_window->get_native_window(), _window->get_native_context());
    ImGui_ImplOpenGL3_Init(glsl_version);

    // OpenCV init

    _use_dashcam = false;
    _dashcam_img = cv::Mat::ones(cv::Size(20, 20), CV_8UC3);
    _arena_img = cv::Mat::ones(cv::Size(20, 20), CV_8UC3);
    _flip_image = false;
    _arena_mouse_pos = ImVec2(0, 0);
    _hsv_slider_names = {
            "Hue (lower)",
            "Hue (upper)",
            "Saturation (lower)",
            "Saturation (upper)",
            "Value (lower)",
            "Value (upper)",
    };
    _pointer_hsv_thresholds = {
            &_hsv_threshold_low[0],
            &_hsv_threshold_high[0],
            &_hsv_threshold_low[1],
            &_hsv_threshold_high[1],
            &_hsv_threshold_low[2],
            &_hsv_threshold_high[2],
    };

    // control init
    if (!_autonomous.init(&_dashcam_img, &_arena_raw_img)) {
        spdlog::error("Error during CAutoController init.");
        exit(-1);
    }
    _auto = false;
    _step = 0;

    _joystick = std::vector<cv::Point>(2, cv::Point(0, 0));

    _hsv_threshold_low = _autonomous.get_hsv_threshold_low();
    _hsv_threshold_high = _autonomous.get_hsv_threshold_high();

    // smaller rot value = ccw, larger rot value = cw
    _waypoints = {
            CAutoController::waypoint{     // WAYPOINT 0
                    cv::Point(0, 0),
                    0,
                    0,
                    false
            },
            CAutoController::waypoint{     // WAYPOINT 1
                    cv::Point(96, 369),
                    14000,
                    0,
                    false
            },
            CAutoController::waypoint{     // WAYPOINT 2 (south target) (fan favourite)
                    cv::Point(245, 450),
                    14000,
                    342,
                    true
            },
            CAutoController::waypoint{     // WAYPOINT 3
                    cv::Point(136, 262),
                    15000,
                    90,
                    true
            },
            CAutoController::waypoint{     // WAYPOINT 4
                    cv::Point(137, 130),
                    14000,
                    70,
                    false
            },
            CAutoController::waypoint{     // WAYPOINT 5
                    cv::Point(327, 115),
                    14000,
                    210,
                    true
            },
            CAutoController::waypoint{     // WAYPOINT 6
                    cv::Point(511, 147),
                    14000,
                    180,
                    true
            },
            CAutoController::waypoint{     // WAYPOINT 7
//                cv::Point(515, 350),
                    cv::Point(458, 334),
                    15000,
                    270,
                    true
            },
            CAutoController::waypoint{     // WAYPOINT 8
                    cv::Point(578, 421),
                    14000,
                    270,
                    false
            },
            CAutoController::waypoint{     // WAYPOINT 9
                    cv::Point(572, 535),
                    20000,
                    270,
                    false
            },
    };

    // preallocate texture handle
    glGenTextures(1, &_dashcam_tex);
    glGenTextures(1, &_arena_tex);

//    // net init
//    // TODO: thread network update separately from

    _udp_req_ready = false;
    _tcp_req_ready = false;

    // start udp update thread
    _thread_update_udp = std::thread(thread_update_udp, this);
    _thread_update_udp.detach();

    // start tcp update thread
    _thread_update_tcp = std::thread(thread_update_tcp, this);
    _thread_update_tcp.detach();
}

CZoomyClient::~CZoomyClient() = default;

void CZoomyClient::update() {

    if (_use_dashcam) {
        // if video capture not set up, connect here
        if (!_video_capture.isOpened()) {
            _dashcam_gst_string = "udpsrc port=5200 ! watchdog timeout=1000 ! application/x-rtp, media=video, clock-rate=90000, payload=96 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink";
            // attempt to connect to udp source, timeout at 1 second
            _video_capture = cv::VideoCapture(_dashcam_gst_string,cv::CAP_GSTREAMER);
        }

        // if source still not opened (timeout reached), default source to videotestsrc
        if (!_video_capture.isOpened()) {
            spdlog::warn("Could not open gstreamer pipeline. Defaulting to videotestsrc");
            _dashcam_gst_string = "videotestsrc ! appsink";
            _video_capture = cv::VideoCapture(_dashcam_gst_string,cv::CAP_GSTREAMER);
        }

        _video_capture.read(_dashcam_raw_img);

        _autonomous.set_hsv_threshold_low(_hsv_threshold_low);
        _autonomous.set_hsv_threshold_high(_hsv_threshold_high);

        if (_flip_image) {
            cv::rotate(_dashcam_raw_img, _dashcam_raw_img, cv::ROTATE_180);
        }

        if (!_dashcam_raw_img.empty()) {
            _detector_params = cv::aruco::DetectorParameters();
            _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            _detector.setDetectorParameters(_detector_params);
            _detector.setDictionary(_dictionary);
            _detector.detectMarkers(_dashcam_raw_img, _marker_corners, _marker_ids, _rejected_candidates);
        }
        if (!_dashcam_raw_img.empty()) cv::aruco::drawDetectedMarkers(_dashcam_raw_img, _marker_corners, _marker_ids);
        _dashcam_img = _dashcam_raw_img;
    } else {
        _video_capture.release();
    }
    //_arena_img = _arena_raw_img;

    if (_values.at(value_type::GC_Y)) {
        _auto = true;
        _step = 0;
    }
    if (_values.at(value_type::GC_B)) {
        _auto = false;
        _values.at(value_type::GC_A) = 0;
        _autonomous.endAutoTarget();
        _autonomous.endRunToPoint();
    }

    if (!_autonomous.isRunning() && _auto) {
        switch (_step) {
            case 0:
                _step++;
                break;
            default:
                if (_step < _waypoints.size()) {
                    _autonomous.startRunToPoint(_waypoints.at(_step).coordinates, _waypoints.at(_step).speed);
                    _values.at(value_type::GC_LTRIG) = _waypoints.at(_step).rotation;
                    _values.at(value_type::GC_A) = _waypoints.at(_step).turret;
                    _step++;
                } else {
                    _auto = false;
                }
                break;
        }
    }
}

void CZoomyClient::draw() {
    float delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _deltaTime).count() / 16.0;
    _deltaTime = std::chrono::steady_clock::now();
    // handle all events
    while (SDL_PollEvent(&_evt)) {
        ImGui_ImplSDL2_ProcessEvent(&_evt);
        switch (_evt.type) {
            case SDL_QUIT:
                spdlog::info("Quit");
                _do_exit = true;
                break;
            case SDL_CONTROLLERBUTTONDOWN:
            case SDL_CONTROLLERBUTTONUP:
                //_values.at(value_type::GC_A) = SDL_GameControllerGetButton(_gc, SDL_CONTROLLER_BUTTON_A);
                _values.at(value_type::GC_B) = SDL_GameControllerGetButton(_gc, SDL_CONTROLLER_BUTTON_B);
                _values.at(value_type::GC_X) = SDL_GameControllerGetButton(_gc, SDL_CONTROLLER_BUTTON_X);
                _values.at(value_type::GC_Y) = SDL_GameControllerGetButton(_gc, SDL_CONTROLLER_BUTTON_Y);
                break;
            case SDL_CONTROLLERAXISMOTION:
                _joystick[0].x = SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_LEFTX);
                _joystick[0].y = SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_LEFTY);
                _joystick[1].x = SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_RIGHTX);
                _joystick[1].y = SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_RIGHTY);

                if (hypot(_joystick[0].x, _joystick[0].y) > DEADZONE) {
                    if (_demo) {
                        _values.at(value_type::GC_LEFTX) = _joystick[0].x * DEMO_SPEED;
                        _values.at(value_type::GC_LEFTY) = _joystick[0].y * DEMO_SPEED;
                    }
                    else {
                        _values.at(value_type::GC_LEFTX) = _joystick[0].x;
                        _values.at(value_type::GC_LEFTY) = _joystick[0].y;
                    }
                } else if (_auto) {
                    _values.at(value_type::GC_LEFTX) = _autonomous.getAutoInput(CAutoController::MOVE_X);
                    _values.at(value_type::GC_LEFTY) = _autonomous.getAutoInput(CAutoController::MOVE_Y);
                } else {
                    _values.at(value_type::GC_LEFTX) = 0;
                    _values.at(value_type::GC_LEFTY) = 0;
                }

                if (_angle > 360.0)
                    _angle -= 360.0;
                else if (_angle < 0.0)
                    _angle += 360.0;

                if (hypot(_joystick[1].x, _joystick[1].y) > DEADZONE) {
                    // look away this won't be pretty...
                    _angle += delta * _joystick[1].x / 32768.0;
                    // nevermind, doesn't look that bad
                    _values.at(value_type::GC_RIGHTX) = _joystick[1].x;
                    _values.at(value_type::GC_RIGHTY) = _joystick[1].y;
                } else if (_auto) {
                    _values.at(value_type::GC_RIGHTX) = _autonomous.getAutoInput(CAutoController::ROTATE);
                    _values.at(value_type::GC_RIGHTY) = 0;
                } else {
                    _values.at(value_type::GC_RIGHTX) = 0;
                    _values.at(value_type::GC_RIGHTY) = 0;
                }

                _values.at(value_type::GC_RTRIG) = SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
                if (!_auto) {
                    _values.at(value_type::GC_LTRIG) = _angle;
                }
                break;
            default:
                break;
        }
    }

    bool *p_open = nullptr;
    ImGuiIO &io = ImGui::GetIO();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::DockSpaceOverViewport();

    // networking settings
    ImGui::Begin("Settings", p_open);
    ImGui::SeparatorText("Networking");
    static char udp_host[64] = "192.168.1.104";
    static char udp_port[64] = "46188";
    static char tcp_host[64] = "192.168.1.156";
    static char tcp_port[64] = "4006";
    ImGui::BeginDisabled(_udp_req_ready);
    ImGui::Text("UDP Host:");
    ImGui::SameLine();
    ImGui::InputText("###udp_host_input", udp_host, 64);
    ImGui::Text("UDP Port:");
    ImGui::SameLine();
    ImGui::InputText("###udp_port_input", udp_port, 64);
    if (ImGui::Button("Connect to UDP")) {
        _udp_host = udp_host;
        _udp_port = udp_port;
        _udp_req_ready = true;
    }
    ImGui::EndDisabled();

    ImGui::BeginDisabled(_tcp_req_ready);
    ImGui::Text("TCP Host:");
    ImGui::SameLine();
    ImGui::InputText("###tcp_host_input", tcp_host, 64);
    ImGui::Text("TCP Port:");
    ImGui::SameLine();
    ImGui::InputText("###tcp_port_input", tcp_port, 64);
    if (ImGui::Button("Connect to TCP")) {
        _tcp_host = tcp_host;
        _tcp_port = tcp_port;
        _tcp_req_ready = true;
    }
    ImGui::EndDisabled();

    ImGui::BeginGroup();
    ImGui::Checkbox("Use dashcam", &_use_dashcam);
    ImGui::Checkbox("Rotate dashcam 180", &_flip_image);
    ImGui::EndGroup();

    std::stringstream ss;
    for (auto &i: _values) {
        ss << i << " ";
    }
    ImGui::Text("%s", ("Values to be sent: " + ss.str()).c_str());

    // opencv parameters
    ImGui::SeparatorText("OpenCV");
    ImGui::Text("Markers: %ld", _marker_ids.size());
    ImGui::BeginGroup();
    ImGui::BeginTable("##cal_item_table", 2, ImGuiTableFlags_SizingFixedFit);
    ImGui::TableSetupColumn("##cal_item_title", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("##cal_item_value", ImGuiTableColumnFlags_WidthStretch);
    for (int i = 0; i < _hsv_slider_names.size(); i++) {
        if (i < 2) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", _hsv_slider_names.at(i).c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::PushItemWidth(-FLT_MIN);
            ImGui::SliderInt(_hsv_slider_names.at(i).c_str(), _pointer_hsv_thresholds.at(i), 0, 180);
            ImGui::PopItemWidth();
        } else {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%s", _hsv_slider_names.at(i).c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::PushItemWidth(-FLT_MIN);
            ImGui::SliderInt(_hsv_slider_names.at(i).c_str(), _pointer_hsv_thresholds.at(i), 0, 255);
            ImGui::PopItemWidth();
        }
    }
    ImGui::EndTable();
    ImGui::EndGroup();

    ImGui::End();

    ImGui::Begin("Waypoints");
    if (ImGui::BeginTable("##waypoints", 4, (ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))) {
        ImGui::TableSetupColumn("X##waypoints_x", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Y##waypoints_y", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Speed##waypoints_speed", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Rotation##waypoints_rotation", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        for (auto &i: _waypoints) {
            ImGui::TableNextRow();
            // X
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", i.coordinates.x);

            // Y
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%d", i.coordinates.y);

            // Speed
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%d", i.speed);

            // Rotation
            ImGui::TableSetColumnIndex(3);
            ImGui::PushItemWidth(-FLT_MIN);
            ImGui::Text("%d", i.rotation);
            ImGui::PopItemWidth();
        }
        ImGui::EndTable();
    }
    ImGui::End();

    // dashcam image
    ImGui::Begin("Dashcam", p_open, ImGuiWindowFlags_MenuBar);
    if (ImGui::BeginMenuBar()) {
        ImGui::MenuItem(_use_dashcam ? _dashcam_gst_string.c_str() : "none",nullptr,false,false);
        ImGui::EndMenuBar();
    }

    // from https://www.reddit.com/r/opengl/comments/114lxvr/imgui_viewport_texture_not_fitting_scaling_to/
    ImVec2 viewport_size = ImGui::GetContentRegionAvail();
    float ratio = ((float) _dashcam_img.cols) / ((float) _dashcam_img.rows);
    float viewport_ratio = viewport_size.x / viewport_size.y;

    _lockout_dashcam.lock();
    mat_to_tex(_dashcam_img, _dashcam_tex);

    // Scale the image horizontally if the content region is wider than the image
    if (viewport_ratio > ratio) {
        float imageWidth = viewport_size.y * ratio;
        float xPadding = (viewport_size.x - imageWidth) / 2;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + xPadding);
        ImGui::Image((ImTextureID) (intptr_t) _dashcam_tex, ImVec2(imageWidth, viewport_size.y));
    }
        // Scale the image vertically if the content region is taller than the image
    else {
        float imageHeight = viewport_size.x / ratio;
        float yPadding = (viewport_size.y - imageHeight) / 2;
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + yPadding);
        ImGui::Image((ImTextureID) (intptr_t) _dashcam_tex, ImVec2(viewport_size.x, imageHeight));
    }

    _lockout_dashcam.unlock();
    ImGui::End();

    // arena image
    ImGui::Begin("Arena", p_open);

    // from https://www.reddit.com/r/opengl/comments/114lxvr/imgui_viewport_texture_not_fitting_scaling_to/
    viewport_size = ImGui::GetContentRegionAvail();
    ratio = ((float) _arena_img.cols) / ((float) _arena_img.rows);
    viewport_ratio = viewport_size.x / viewport_size.y;

//    if (_autonomous.isRunning()) {
//        cv::Mat temp_img = _autonomous.get_masked_image();
//        mat_to_tex(temp_img, _arena_tex);
//    } else {
    mat_to_tex(_arena_raw_img, _arena_tex);
//    }

    // Scale the image horizontally if the content region is wider than the image
    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float scaled_size = 0.0f;
    float how_much_to_scale_coordinates = 0.0f;
    if (viewport_ratio > ratio) {
        float imageWidth = viewport_size.y * ratio;
        float xPadding = (viewport_size.x - imageWidth) / 2;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + xPadding);
        pos_x = ImGui::GetCursorScreenPos().x;
        pos_y = ImGui::GetCursorScreenPos().y;
        ImGui::Image((ImTextureID) (intptr_t) _arena_tex, ImVec2(imageWidth, viewport_size.y));
        scaled_size = imageWidth;
        how_much_to_scale_coordinates = ARENA_DIM / scaled_size;
    }
        // Scale the image vertically if the content region is taller than the image
    else {
        float imageHeight = viewport_size.x / ratio;
        float yPadding = (viewport_size.y - imageHeight) / 2;
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + yPadding);
        pos_x = ImGui::GetCursorScreenPos().x;
        pos_y = ImGui::GetCursorScreenPos().y;
        ImGui::Image((ImTextureID) (intptr_t) _arena_tex, ImVec2(viewport_size.x, imageHeight));
        scaled_size = imageHeight;
        how_much_to_scale_coordinates = ARENA_DIM / scaled_size;
    }

    if (ImGui::IsItemHovered()) {
        ImVec2 arena_mouse_pos = ImVec2((ImGui::GetMousePos().x - pos_x) * how_much_to_scale_coordinates,
                                        (ImGui::GetMousePos().y - pos_y) * how_much_to_scale_coordinates);
        _arena_mouse_pos.x = arena_mouse_pos.x < 0 ? 0 : arena_mouse_pos.x > ARENA_DIM ? ARENA_DIM : arena_mouse_pos.x;
        _arena_mouse_pos.y = arena_mouse_pos.y < 0 ? 0 : arena_mouse_pos.y > ARENA_DIM ? ARENA_DIM : arena_mouse_pos.y;
    }

    ImGui::End();

    // imgui window (for debug)
    ImGui::Begin("ImGui", p_open);
    ImGui::Text("dear imgui says hello! (%s) (%d)", IMGUI_VERSION, IMGUI_VERSION_NUM);
    ImGui::Text("Arena mouse position: %d %d", (int) _arena_mouse_pos.x, (int) _arena_mouse_pos.y);
    ImGui::SeparatorText("OpenCV Build Information");
    ImGui::Text("%s", cv::getBuildInformation().c_str());

    ImGui::End();

    ImGui::Render();

    glViewport(0, 0, static_cast<int>(io.DisplaySize.x), static_cast<int>(io.DisplaySize.y));
    glClearColor(0.5F, 0.5F, 0.5F, 1.00F);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if ((io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) != 0) {
        SDL_Window *backup_current_window{SDL_GL_GetCurrentWindow()};
        SDL_GLContext backup_current_context{SDL_GL_GetCurrentContext()};
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        SDL_GL_MakeCurrent(backup_current_window, backup_current_context);
    }

    SDL_GL_SwapWindow(_window->get_native_window());

    // limit to 1000 FPS
    while ((int) std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - _perf_draw_start).count() < 1);
}

void CZoomyClient::udp_rx() {
    _udp_rx_bytes = 0;
    _udp_rx_buf.clear();
    _udp_client.do_rx(_udp_rx_buf, _udp_rx_bytes);
    std::vector<uint8_t> temp(_udp_rx_buf.begin(), _udp_rx_buf.begin() + _udp_rx_bytes);
    // only add to udp_rx queue if data is not empty and not ping response
    if (!temp.empty() && (temp.front() != '\6')) _udp_rx_queue.emplace(temp);
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(1));
}

void CZoomyClient::udp_tx() {
    for (; !_udp_tx_queue.empty(); _udp_tx_queue.pop()) {
//        spdlog::info("Sending" + std::string(_udp_tx_queue.front().begin(), _udp_tx_queue.front().end()));
        _udp_client.do_tx(_udp_tx_queue.front());
    }
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyClient::update_udp() {
    if (!_udp_client.get_socket_status()) {
        if (_udp_req_ready) {
            _udp_client.ping();
            _udp_client.setup(_udp_host, _udp_port);

            _udp_timeout_count = std::chrono::steady_clock::now();
            _udp_send_data = _udp_client.get_socket_status();

            // start listen thread
            _thread_udp_rx = std::thread(thread_udp_rx, this);
            _thread_udp_rx.detach();

            // start send thread
            _thread_udp_tx = std::thread(thread_udp_tx, this);
            _thread_udp_tx.detach();
        }
        std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
    } else {
        for (; !_udp_rx_queue.empty(); _udp_rx_queue.pop()) {

//            // acknowledge next data in queue
            spdlog::info("New in RX queue with size: " + std::to_string(_udp_rx_queue.front().size()));

            // reset timeout
            // placement of this may be a source of future bug
            _udp_timeout_count = std::chrono::steady_clock::now();
        }
        // do stuff...
        std::string payload;
        for (auto &i: _values) {
            payload += std::to_string(i) + " ";
        }

        _udp_tx_queue.emplace(payload.begin(), payload.end());
        spdlog::info("Last response time (ms): " + std::to_string(_udp_client.get_last_response_time()));
        std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
    }
}

void CZoomyClient::thread_update_udp(CZoomyClient *who_called) {
    while (!who_called->_do_exit) {
        who_called->update_udp();
    }
}

void CZoomyClient::thread_udp_rx(CZoomyClient *who_called) {
    while (who_called->_udp_client.get_socket_status()) {
        who_called->udp_rx();
    }
}

void CZoomyClient::thread_udp_tx(CZoomyClient *who_called) {
    while (who_called->_udp_client.get_socket_status()) {
        who_called->udp_tx();
    }
}

void CZoomyClient::tcp_rx() {
    _tcp_rx_bytes = 0;
    _tcp_rx_buf.clear();
    _tcp_client.do_rx(_tcp_rx_buf, _tcp_rx_bytes);
    std::vector<uint8_t> temp(_tcp_rx_buf.begin(), _tcp_rx_buf.begin() + _tcp_rx_bytes);
    // only add to tcp_rx queue if data is not empty and not ping response
    if (!temp.empty() && (temp.front() != '\6')) _tcp_rx_queue.emplace(temp);
    // don't check for new packets too often
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(TCP_DELAY));

}

void CZoomyClient::tcp_tx() {
    for (; !_tcp_tx_queue.empty(); _tcp_tx_queue.pop()) {
//        spdlog::info("Sending" + std::string(_tcp_tx_queue.front().begin(), _tcp_tx_queue.front().end()));
        _tcp_client.do_tx(_tcp_tx_queue.front());
    }
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(TCP_DELAY));
}

void CZoomyClient::update_tcp() {
    if (!_tcp_client.get_socket_status()) {
        if (_tcp_req_ready) {
            _tcp_client.setup(_tcp_host, _tcp_port);
            _tcp_send_data = _tcp_client.get_socket_status();

            // start listen thread
            _thread_tcp_rx = std::thread(thread_tcp_rx, this);
            _thread_tcp_rx.detach();

            // start send thread
            _thread_tcp_tx = std::thread(thread_tcp_tx, this);
            _thread_tcp_tx.detach();
        }
        std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
    } else {
        for (; !_tcp_rx_queue.empty(); _tcp_rx_queue.pop()) {
//            // acknowledge next data in queue
            spdlog::info("New in RX queue with size: " + std::to_string(_tcp_rx_queue.front().size()));
            cv::imdecode(_tcp_rx_queue.front(), cv::IMREAD_UNCHANGED, &_arena_raw_img);
        }
        std::string payload = "G 1";
        _tcp_tx_queue.emplace(payload.begin(), payload.end());
        std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(TCP_DELAY));
    }
}

void CZoomyClient::thread_update_tcp(CZoomyClient *who_called) {
    while (!who_called->_do_exit) {
        who_called->update_tcp();
    }
}

void CZoomyClient::thread_tcp_rx(CZoomyClient *who_called) {
    while (who_called->_tcp_client.get_socket_status()) {
        who_called->tcp_rx();
    }
}

void CZoomyClient::thread_tcp_tx(CZoomyClient *who_called) {
    while (who_called->_tcp_client.get_socket_status()) {
        who_called->tcp_tx();
    }
}

void CZoomyClient::mat_to_tex(cv::Mat &input, GLuint &output) {
    if (input.empty()) return;
    cv::Mat flipped;
    // might crash here if input array is somehow emptied
    cv::cvtColor(input, flipped, cv::COLOR_BGR2RGB);

    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                    GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

    glBindTexture(GL_TEXTURE_2D, output);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, input.cols, input.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, flipped.data);
}

int main(int argc, char *argv[]) {
    CZoomyClient c = CZoomyClient(cv::Size(1280, 720));
    c.run();
    return 0;
}