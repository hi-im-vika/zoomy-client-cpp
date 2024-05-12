/**
 * CZoomyClient.cpp - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CZoomyClient.hpp"

#define PING_TIMEOUT 1000
#define NET_DELAY 35

CZoomyClient::CZoomyClient(cv::Size s, std::string host, std::string port) {
    _window_size = s;

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

    _steering_trim = _throttle_trim = 0;
    _values = {0, 0, 0, 0, 0, 0, 0, 0, 0 ,0};

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

    io.ConfigFlags |=
            ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_ViewportsEnable | ImGuiConfigFlags_DockingEnable;
//    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
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

    _video_capture = cv::VideoCapture("udpsrc port=5200 ! application/x-rtp, media=video, clock-rate=90000, payload=96 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink", cv::CAP_GSTREAMER);
    _dashcam_img = cv::Mat::ones(cv::Size(20, 20), CV_8UC3);
    _arena_img = cv::Mat::ones(cv::Size(20, 20), CV_8UC3);
    _flip_image = false;

    // preallocate texture handle

    glGenTextures(1, &_dashcam_tex);
    glGenTextures(1, &_arena_tex);

//    // net init
//    // TODO: thread network update separately from
//
//    _udp_host = host;
//    _udp_port = port;
//    _udp_client.setup(_udp_host,_udp_port);
//
//    _udp_timeout_count = std::chrono::steady_clock::now();
//    _udp_send_data = _udp_client.get_socket_status();
//
//    // start listen thread
//    _thread_udp_rx = std::thread(thread_udp_rx, this);
//    _thread_udp_rx.detach();
//
//    // start send thread
//    _thread_udp_tx = std::thread(thread_udp_tx, this);
//    _thread_udp_tx.detach();
}

CZoomyClient::~CZoomyClient() = default;

void CZoomyClient::update() {
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
    _arena_img = _arena_raw_img;
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
            case SDL_CONTROLLERBUTTONDOWN:
            case SDL_CONTROLLERBUTTONUP:
                _values.at(value_type::GC_A) = SDL_GameControllerGetButton(_gc, SDL_CONTROLLER_BUTTON_A);
                _values.at(value_type::GC_B) = SDL_GameControllerGetButton(_gc, SDL_CONTROLLER_BUTTON_B);
                _values.at(value_type::GC_X) = SDL_GameControllerGetButton(_gc, SDL_CONTROLLER_BUTTON_X);
                _values.at(value_type::GC_Y) = SDL_GameControllerGetButton(_gc, SDL_CONTROLLER_BUTTON_Y);
                break;
            case SDL_CONTROLLERAXISMOTION:
                if (_do_invert_steering) {
                    _values.at(value_type::GC_LEFTX) = normalize_with_trim((SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_LEFTX) + _steering_trim) * -1, _steering_trim);
                } else {
                    _values.at(value_type::GC_LEFTX) = normalize_with_trim((SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_LEFTX) + _steering_trim), _steering_trim);
                }
                _values.at(value_type::GC_LEFTY) = SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_LEFTY);
                _values.at(value_type::GC_RIGHTX) = SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_RIGHTX);
                _values.at(value_type::GC_RIGHTY) = normalize_with_trim((SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_RIGHTY) + _throttle_trim), _throttle_trim);
                _values.at(value_type::GC_LTRIG) = SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
                _values.at(value_type::GC_RTRIG) = SDL_GameControllerGetAxis(_gc, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
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

    ImGui::Begin("Connect", p_open);
    ImGui::Text("Host:");
    ImGui::SameLine();
    ImGui::InputText("###host_input", _udp_host.data(), _udp_host.capacity());
    ImGui::Text("Port:");
    ImGui::SameLine();
    ImGui::InputText("###port_input", _udp_port.data(), _udp_port.capacity());
    std::stringstream ss;
    for (auto &i : _values) {
        ss << i << " ";
    }
    ImGui::Text("%s", ("Values to be sent: " + ss.str()).c_str());
    ImGui::End();

    ImGui::Begin("OpenCV", p_open);

    // from https://www.reddit.com/r/opengl/comments/114lxvr/imgui_viewport_texture_not_fitting_scaling_to/
    ImVec2 viewport_size = ImGui::GetContentRegionAvail();
    float ratio = ((float) _img.cols) / ((float) _img.rows);
    float viewport_ratio = viewport_size.x / viewport_size.y;

    _lockout.lock();
    mat_to_tex(_img, _tex);

    // Scale the image horizontally if the content region is wider than the image
    if (viewport_ratio > ratio) {
        float imageWidth = viewport_size.y * ratio;
        float xPadding = (viewport_size.x - imageWidth) / 2;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + xPadding);
        ImGui::Image((ImTextureID) (intptr_t) _tex, ImVec2(imageWidth, viewport_size.y));
    }
        // Scale the image vertically if the content region is taller than the image
    else {
        float imageHeight = viewport_size.x / ratio;
        float yPadding = (viewport_size.y - imageHeight) / 2;
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + yPadding);
        ImGui::Image((ImTextureID) (intptr_t) _tex, ImVec2(viewport_size.x, imageHeight));
    }

    _lockout.unlock();
    ImGui::BeginGroup();
    ImGui::Button("Connect to AVFoundation");
    ImGui::SameLine();
    ImGui::Button("Connect to GStreamer");
    ImGui::SameLine();
    ImGui::Checkbox("Rotate 180",&_flip_image);
    ImGui::EndGroup();
    ImGui::End();

    ImGui::Begin("OpenCV Details", p_open);
    ImGui::Text("Markers: %ld", _marker_ids.size());
    ImGui::End();

    ImGui::Begin("ImGui", p_open);
    ImGui::Text("dear imgui says hello! (%s) (%d)", IMGUI_VERSION, IMGUI_VERSION_NUM);

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

void CZoomyClient::rx() {
    _udp_rx_bytes = 0;
    _udp_rx_buf.clear();
    _udp_client.do_rx(_udp_rx_buf, _udp_rx_bytes);
    std::vector<uint8_t> temp(_udp_rx_buf.begin(), _udp_rx_buf.begin() + _udp_rx_bytes);
    // only add to rx queue if data is not empty and not ping response
    if(!temp.empty() && (temp.front() != '\6')) _udp_rx_queue.emplace(temp);
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(1));
}

void CZoomyClient::tx() {
    for (; !_udp_tx_queue.empty(); _udp_tx_queue.pop()) {
//        spdlog::info("Sending" + std::string(_udp_tx_queue.front().begin(), _udp_tx_queue.front().end()));
        _udp_client.do_tx(_udp_tx_queue.front());
    }
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyClient::thread_udp_rx(CZoomyClient *who_called) {
    while (!who_called->_udp_ready) {
        who_called->rx();
    }
}

void CZoomyClient::thread_udp_tx(CZoomyClient *who_called) {
    while (!who_called->_udp_ready) {
        who_called->tx();
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

int CZoomyClient::normalize_with_trim(int i, int trim) {
    int mult = i > 0 ? 1 : -1;
    int locked_range = 32767 - trim;
    int raw = i - trim;
    if ((locked_range - abs(raw)) < 0) {
        return (mult * locked_range) + trim;
    } else {
        return i;
    }
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: client <host> <port>" << std::endl;
        return 1;
    }

    CZoomyClient c = CZoomyClient(cv::Size(854, 480), argv[1], argv[2]);
    c.run();
    return 0;
}