/**
 * CZoomyClient.cpp - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CZoomyClient.hpp"

#define PING_TIMEOUT 1000
#define NET_DELAY 35
#define DEADZONE 2048
#define ARENA_DIM 600

// increase this value if malloc_error_break happens too often
#define TCP_DELAY 30
//#define TCP_DELAY 15 // only if over ssh forwarding

CZoomyClient::CZoomyClient(cv::Size s) {
    _window_size = s;

    // SDL init
    uint init_flags = SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER;

    if (SDL_Init(init_flags) != 0) {
        spdlog::error("Error during SDL init");
        exit(-1);
    }

//    // control init

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
    _arena_img = cv::Mat::ones(cv::Size(ARENA_DIM, ARENA_DIM), CV_8UC3);
    _arena_mask_img = _arena_img;
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
            _video_capture = cv::VideoCapture(_dashcam_gst_string, cv::CAP_GSTREAMER);
        }

        // if source still not opened (timeout reached), default source to videotestsrc
        if (!_video_capture.isOpened()) {
            spdlog::warn("Could not open gstreamer pipeline. Defaulting to videotestsrc");
            _dashcam_gst_string = "videotestsrc ! appsink";
            _video_capture = cv::VideoCapture(_dashcam_gst_string, cv::CAP_GSTREAMER);
        }

        _video_capture.read(_dashcam_raw_img);

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

    _autonomous.set_hsv_threshold_low(_hsv_threshold_low);
    _autonomous.set_hsv_threshold_high(_hsv_threshold_high);

    if (_show_mask) {
        cv::Mat pregen = _arena_raw_img;
        cv::Mat hsv, inrange, mask, anded;

        cv::cvtColor(pregen, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv,
                    (cv::Scalar) _autonomous.get_hsv_threshold_low(),
                    (cv::Scalar) _autonomous.get_hsv_threshold_high(),
                    inrange);
        inrange.convertTo(mask, CV_8UC1);
        cv::bitwise_and(pregen, pregen, anded, mask);
        _mutex_mask_gen.lock();
        _arena_mask_img = anded;
        _mutex_mask_gen.unlock();
    }

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

    // handle all events
    while (SDL_PollEvent(&_evt)) {
        ImGui_ImplSDL2_ProcessEvent(&_evt);
        switch (_evt.type) {
            case SDL_QUIT:
                spdlog::info("Quit");
                _do_exit = true;
                break;
            case SDL_CONTROLLERDEVICEADDED:
                spdlog::info("GC added");
                break;
            case SDL_CONTROLLERDEVICEREMOVED:
                spdlog::info("GC removed");
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
                    _values.at(value_type::GC_LEFTX) = _joystick[0].x;
                    _values.at(value_type::GC_LEFTY) = _joystick[0].y;
                } else if (_auto) {
                    _values.at(value_type::GC_LEFTX) = _autonomous.getAutoInput(CAutoController::MOVE_X);
                    _values.at(value_type::GC_LEFTY) = _autonomous.getAutoInput(CAutoController::MOVE_Y);
                } else {
                    _values.at(value_type::GC_LEFTX) = 0;
                    _values.at(value_type::GC_LEFTY) = 0;
                }

                if (hypot(_joystick[1].x, _joystick[1].y) > DEADZONE) {
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
                break;
            default:
                break;
        }
    }

    // if viewport is minimized, don't draw
    if (SDL_GetWindowFlags(_window->get_native_window()) & SDL_WINDOW_MINIMIZED) return;

    ImGuiIO &io = ImGui::GetIO();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGui::DockSpaceOverViewport();

    imgui_draw_settings();
    imgui_draw_waypoints();
    imgui_draw_dashcam();
    imgui_draw_arena();
    imgui_draw_debug();

    ImGui::Render();

    // render ImGui with OpenGL
    glViewport(0, 0, static_cast<int>(io.DisplaySize.x), static_cast<int>(io.DisplaySize.y));
    glClearColor(0.5F, 0.5F, 0.5F, 1.00F);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

//    // update other viewports if multi-viewport enabled
//    if ((io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) != 0) {
//        SDL_Window *backup_current_window{SDL_GL_GetCurrentWindow()};
//        SDL_GLContext backup_current_context{SDL_GL_GetCurrentContext()};
//        ImGui::UpdatePlatformWindows();
//        ImGui::RenderPlatformWindowsDefault();
//        SDL_GL_MakeCurrent(backup_current_window, backup_current_context);
//    }

    SDL_GL_SwapWindow(_window->get_native_window());

    // limit to 1000 FPS
    while ((int) std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - _perf_draw_start).count() < 1);
}

void CZoomyClient::imgui_draw_settings() {
    // networking settings
    ImGui::Begin("Settings", nullptr);
    ImGui::SeparatorText("Controls");
    ImGui::Text("Choose gamepad:");
    int joysticks = SDL_NumJoysticks();
    std::vector<std::string> joystick_names;

    // if joysticks are connected
    if (joysticks) {
        // get all joystick names
        for (int i = 0; i < joysticks; i++) {
            joystick_names.emplace_back(SDL_GameControllerNameForIndex(i));
        }
        // if no gc assigned already, assign first one
        if (!_gc) _gc = SDL_GameControllerFromInstanceID(SDL_JoystickGetDeviceInstanceID(0));
    } else {
        // if nothing connected, set gc to nullptr
        _gc = nullptr;
    }

    int item_selected_idx = 0;
    std::string combo_preview_value = joysticks ? SDL_GameControllerName(_gc) : "No gamepads connected";

    ImGui::BeginDisabled(!joysticks);
    ImGui::PushItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##gpselect", combo_preview_value.c_str())) {
        for (int i = 0; i < joysticks; i++) {
            const bool is_selected = (item_selected_idx == i);
            if (ImGui::Selectable(SDL_GameControllerNameForIndex(i), is_selected)) {
                item_selected_idx = i;
                _gc = SDL_GameControllerFromInstanceID(SDL_JoystickGetDeviceInstanceID(i));
            }
            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    ImGui::PopItemWidth();
    ImGui::EndDisabled();

    ImGui::SeparatorText("Networking");

    // placeholder values
    static char udp_host[64] = "192.168.1.104";
    static char udp_port[64] = "46188";
    static char tcp_host[64] = "127.0.0.1";
    static char tcp_port[64] = "4006";

    ImGui::BeginGroup();

    // draw udp conn details table
    ImGui::BeginDisabled(_udp_req_ready);
    ImGui::BeginTable("##udp_item_table", 2, ImGuiTableFlags_SizingFixedFit);
    ImGui::TableSetupColumn("##udp_item_title", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("##udp_item_value", ImGuiTableColumnFlags_WidthStretch);

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("UDP Host:");
    ImGui::TableSetColumnIndex(1);
    ImGui::PushItemWidth(-FLT_MIN);
    ImGui::InputText("###udp_host_input", udp_host, 64);
    ImGui::PopItemWidth();

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("UDP Port:");
    ImGui::TableSetColumnIndex(1);
    ImGui::PushItemWidth(-FLT_MIN);
    ImGui::InputText("###udp_port_input", udp_port, 64);
    ImGui::PopItemWidth();
    ImGui::EndTable();

    ImGui::PushItemWidth(-FLT_MIN);
    if (ImGui::Button("Connect to UDP")) {
        _udp_host = udp_host;
        _udp_port = udp_port;
        _udp_req_ready = true;
    }
    ImGui::PopItemWidth();
    ImGui::EndDisabled();

    // draw udp conn details table
    ImGui::BeginDisabled(_tcp_req_ready);
    ImGui::BeginTable("##tcp_item_table", 2, ImGuiTableFlags_SizingFixedFit);
    ImGui::TableSetupColumn("##tcp_item_title", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("##tcp_item_value", ImGuiTableColumnFlags_WidthStretch);

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("TCP Host:");
    ImGui::TableSetColumnIndex(1);
    ImGui::PushItemWidth(-FLT_MIN);
    ImGui::InputText("###tcp_host_input", tcp_host, 64);
    ImGui::PopItemWidth();

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("TCP Port:");
    ImGui::TableSetColumnIndex(1);
    ImGui::PushItemWidth(-FLT_MIN);
    ImGui::InputText("###tcp_port_input", tcp_port, 64);
    ImGui::PopItemWidth();
    ImGui::EndTable();

    ImGui::PushItemWidth(-FLT_MIN);
    if (ImGui::Button("Connect to TCP")) {
        _tcp_host = tcp_host;
        _tcp_port = tcp_port;
        _tcp_req_ready = true;
    }
    ImGui::PopItemWidth();
    ImGui::EndDisabled();
    ImGui::EndGroup();

    // draw checkboxes for toggleable values
    ImGui::BeginGroup();
    ImGui::Checkbox("Use dashcam", &_use_dashcam);
    ImGui::Checkbox("Rotate dashcam 180", &_flip_image);
    ImGui::EndGroup();

    // TODO: determine maximum number of values to send and remove stringstream
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
}

void CZoomyClient::imgui_draw_waypoints() {
    ImGui::Begin("Waypoints");
    if (ImGui::BeginTable("##waypoints", 4,
                          (ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))) {
        ImGui::TableSetupColumn("X##waypoints_x", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Y##waypoints_y", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Speed##waypoints_speed", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Rotation##waypoints_rotation", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        int wp_id = 0;  // keep track of current waypoint
        char label[32];
        bool was_hovered = false;   // remember if row inside table was hovered
        for (auto &i: _waypoints) {
            snprintf(label, 32, "##waypoint_%d", wp_id);
            ImGui::PushID(label);
            ImGui::TableNextRow();
            // X
            ImGui::TableSetColumnIndex(0);
            snprintf(label, 32, "%d", i.coordinates.x);
            bool a = false;

            ImGuiSelectableFlags selectable_flags =
                    ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap;
            ImGui::Selectable(label, a, selectable_flags);

            if (ImGui::IsItemHovered()) {
                _wp_highlighted = wp_id;
                was_hovered = true;
            }
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

            ImGui::PopID();
            wp_id++;
        }
        if (!was_hovered) _wp_highlighted = -1; // if nothing was hovered, set highlight to false
        ImGui::EndTable();
    }
    ImGui::End();
}

void CZoomyClient::imgui_draw_dashcam() {
    // dashcam image
    ImGui::Begin("Dashcam", nullptr, ImGuiWindowFlags_MenuBar);
    if (ImGui::BeginMenuBar()) {
        ImGui::MenuItem(_use_dashcam ? _dashcam_gst_string.c_str() : "none", nullptr, false,
                        false);
        ImGui::EndMenuBar();
    }

    _mutex_dashcam.lock();
    fit_texture_to_window(_dashcam_img, _dashcam_tex);
    _mutex_dashcam.unlock();

    ImGui::End();
}

void CZoomyClient::imgui_draw_arena() {
    // arena image
    ImGui::Begin("Arena", nullptr, ImGuiWindowFlags_MenuBar);

    if (ImGui::BeginMenuBar()) {
        ImGui::BeginDisabled(_arena_raw_img.empty());
        ImGui::Checkbox("Overlay Mask", &_show_mask);
        ImGui::EndDisabled();
        ImGui::EndMenuBar();
    }

    // copy out latest arena image
    if (_show_mask) {
        _mutex_mask_gen.lock();
        _arena_img = _arena_mask_img;
        _mutex_mask_gen.unlock();
    } else {
        _arena_img = _arena_raw_img;
    }

    float scaled_factor = 0.0f;
    ImVec2 last_cursor_pos;
    fit_texture_to_window(_arena_img, _arena_tex, scaled_factor, last_cursor_pos);
    mat_to_tex(_arena_img, _arena_tex);
    float coord_scale = ARENA_DIM / scaled_factor;

    if (ImGui::IsItemHovered()) {
        ImVec2 arena_mouse_pos = ImVec2((ImGui::GetMousePos().x - last_cursor_pos.x) * coord_scale,
                                        (ImGui::GetMousePos().y - last_cursor_pos.y) * coord_scale);
        _arena_mouse_pos.x =
                arena_mouse_pos.x < 0 ? 0 : arena_mouse_pos.x > ARENA_DIM ? ARENA_DIM : arena_mouse_pos.x;
        _arena_mouse_pos.y =
                arena_mouse_pos.y < 0 ? 0 : arena_mouse_pos.y > ARENA_DIM ? ARENA_DIM : arena_mouse_pos.y;
//        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(ImGui::GetMousePos().x, ImGui::GetMousePos().y), 15,
//                                                    ImColor(ImVec4(1.0f, 1.0f, 0.4f, 1.0f)));
    }

    // plot waypoints in ImGui instead of OpenCV
    int wp = 0; // keep track of which waypoint plotted
    ImGui::GetWindowDrawList()->ChannelsSplit(2);
    for (auto &i: _waypoints) {
        ImGui::GetWindowDrawList()->ChannelsSetCurrent(1);

        // modify waypoint coords to fit on image
        ImVec2 pt_ctr = ImVec2(((float) i.coordinates.x / coord_scale) + last_cursor_pos.x,
                               ((float) i.coordinates.y / coord_scale) + last_cursor_pos.y);

        // plot the waypoint
        ImColor wp_colour = wp == _wp_highlighted ? ImColor(ImVec4(1.0f, 0.5f, 0.0f, 1.0f)) : ImColor(
                ImVec4(1.0f, 1.0f, 0.4f, 1.0f));
        ImGui::GetWindowDrawList()->AddCircleFilled(pt_ctr, 10, wp_colour);

        // draw waypoint index on top of waypoint
        ImGui::GetWindowDrawList()->AddText(ImGui::GetFont(), ImGui::GetFontSize(),
                                            ImVec2(pt_ctr.x - (ImGui::GetFontSize() / 4),
                                                   pt_ctr.y - (ImGui::GetFontSize() / 2)), IM_COL32_BLACK,
                                            std::to_string(wp).c_str());

        // draw connecting line
        ImGui::GetWindowDrawList()->ChannelsSetCurrent(0);
        if (wp) {   // if not the first waypoints
            auto last = std::prev(&i);  // get last waypoint
            // modify waypoint coords to fit on image
            ImVec2 last_pt_ctr = ImVec2(((float) last->coordinates.x / coord_scale) + last_cursor_pos.x,
                                        ((float) last->coordinates.y / coord_scale) + last_cursor_pos.y);
            // draw line from prev waypoint to current waypoint
            // maybe add arrow to line
            // ImGui::GetWindowDrawList()->AddNgonFilled(ImVec2(((pt_ctr.x - last_pt_ctr.x) / 2) + last_pt_ctr.x,((pt_ctr.y - last_pt_ctr.y) / 2) + last_pt_ctr.y), 10, wp_colour, 3);
            ImGui::GetWindowDrawList()->AddLine(last_pt_ctr, pt_ctr, ImColor(ImVec4(1.0f, 1.0f, 0.4f, 1.0f)), 3);
        }
        wp++;
    }

    ImGui::End();
}

void CZoomyClient::imgui_draw_debug() {
    // imgui window (for debug)
    ImGui::Begin("ImGui", nullptr);
    ImGui::Text("dear imgui says hello! (%s) (%d)", IMGUI_VERSION, IMGUI_VERSION_NUM);
    ImGui::Text("Arena mouse position: %d %d", (int) _arena_mouse_pos.x, (int) _arena_mouse_pos.y);
    ImGui::Text("Viewport %f %f", ImGui::GetMainViewport()->Size.x, ImGui::GetMainViewport()->Size.y);
    ImGui::SeparatorText("OpenCV Build Information");
    ImGui::Text("%s", cv::getBuildInformation().c_str());
    ImGui::ShowDemoWindow();

    ImGui::End();
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
            // acknowledge next data in queue
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

// only call this from inside imgui window
void CZoomyClient::fit_texture_to_window(cv::Mat &input_image, GLuint &output_texture, float &scale,
                                         ImVec2 &cursor_screen_pos_before_image) {
    // from https://www.reddit.com/r/opengl/comments/114lxvr/imgui_viewport_texture_not_fitting_scaling_to/
    ImVec2 viewport_size = ImGui::GetContentRegionAvail();
    float ratio = ((float) input_image.cols) / ((float) input_image.rows);
    float viewport_ratio = viewport_size.x / viewport_size.y;
    mat_to_tex(input_image, output_texture);

    // Scale the image horizontally if the content region is wider than the image
    if (viewport_ratio > ratio) {
        float imageWidth = viewport_size.y * ratio;
        float xPadding = (viewport_size.x - imageWidth) / 2;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + xPadding);
        cursor_screen_pos_before_image = ImGui::GetCursorScreenPos();
        ImGui::Image((ImTextureID) (intptr_t) output_texture, ImVec2(imageWidth, viewport_size.y));
        scale = imageWidth;
    }
    // Scale the image vertically if the content region is taller than the image
    else {
        float imageHeight = viewport_size.x / ratio;
        float yPadding = (viewport_size.y - imageHeight) / 2;
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + yPadding);
        cursor_screen_pos_before_image = ImGui::GetCursorScreenPos();
        ImGui::Image((ImTextureID) (intptr_t) output_texture, ImVec2(viewport_size.x, imageHeight));
        scale = imageHeight;
    }
}

// only call this from inside imgui window
void CZoomyClient::fit_texture_to_window(cv::Mat &input_image, GLuint &output_texture) {
    float dont_care_float;
    ImVec2 dont_care_imvec;
    fit_texture_to_window(input_image, output_texture, dont_care_float, dont_care_imvec);
}

int main(int argc, char *argv[]) {
    CZoomyClient c = CZoomyClient(cv::Size(1280, 720));
    c.run();
    return 0;
}