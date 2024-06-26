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
    uint init_flags = SDL_INIT_VIDEO | SDL_INIT_TIMER;

    if (SDL_Init(init_flags) != 0) {
        spdlog::error("Error during SDL init");
        exit(-1);
    }

    std::stringstream ss;

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

    _img = cv::Mat::ones(cv::Size(20,20),CV_8UC3);
    _flip_image = false;

    // preallocate texture handle

    glGenTextures(1, &_tex);
    glGenTextures(1, &_another_tex);

    // net init

    _host = host;
    _port = port;
    _client.setup(_host,_port);

    _timeout_count = std::chrono::steady_clock::now();
    _send_data = _client.get_socket_status();

    // start listen thread
    _thread_rx = std::thread(thread_rx, this);
    _thread_rx.detach();

    // start send thread
    _thread_tx = std::thread(thread_tx, this);
    _thread_tx.detach();
}

CZoomyClient::~CZoomyClient() = default;

void CZoomyClient::update() {
    cv::Mat raw_img;
    for (; !_rx_queue.empty(); _rx_queue.pop()) {

//            // acknowledge next data in queue
        spdlog::info("New in RX queue with size: " + std::to_string(_rx_queue.front().size()));
        std::vector<uint8_t> temporary(_rx_queue.front().begin() + 4,_rx_queue.front().end());
        cv::imdecode(temporary, cv::IMREAD_COLOR, &raw_img);

        _lockout.lock();

        if (_flip_image) {
            cv::rotate(raw_img,raw_img,cv::ROTATE_180);
        }

        if (!raw_img.empty()) {
            _detector_params = cv::aruco::DetectorParameters();
            _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            _detector.setDetectorParameters(_detector_params);
            _detector.setDictionary(_dictionary);
            _detector.detectMarkers(raw_img, _marker_corners, _marker_ids, _rejected_candidates);
        }
        if (!raw_img.empty()) cv::aruco::drawDetectedMarkers(raw_img, _marker_corners, _marker_ids);
        _img = raw_img;

        _lockout.unlock();

        // reset timeout
        // placement of this may be a source of future bug
        _timeout_count = std::chrono::steady_clock::now();
    }

    _time_since_start = (int) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _timeout_count).count();
    if (_time_since_start > PING_TIMEOUT) {
        spdlog::warn("Server is gone...");
        _send_data = false;
        do {
            _client.setdn();
            _client.setup(_host, _port);
        } while (!_client.get_socket_status());
        _send_data = _client.get_socket_status();
        _timeout_count = std::chrono::steady_clock::now();
    }

    std::string payload = "asdf";
    _tx_queue.emplace(payload.begin(), payload.end());
    spdlog::info("Last response time (ms): " + std::to_string(_client.get_last_response_time()));
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
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
    ImGui::InputText("###host_input", _host.data(), _host.capacity());
    ImGui::Text("Port:");
    ImGui::SameLine();
    ImGui::InputText("###port_input", _port.data(), _port.capacity());
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
    _rx_bytes = 0;
    _rx_buf.clear();
    _client.do_rx(_rx_buf, _rx_bytes);
    std::vector<uint8_t> temp(_rx_buf.begin(),_rx_buf.begin() + _rx_bytes);
    // only add to rx queue if data is not empty
    if(!temp.empty()) _rx_queue.emplace(temp);
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(1));
}

void CZoomyClient::tx() {
    for (; !_tx_queue.empty(); _tx_queue.pop()) {
//            spdlog::info("Sending");
        _client.do_tx(_tx_queue.front());
    }
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyClient::thread_rx(CZoomyClient *who_called) {
    while (!who_called->_do_exit) {
        who_called->rx();
    }
}

void CZoomyClient::thread_tx(CZoomyClient *who_called) {
    while (!who_called->_do_exit) {
        who_called->tx();
    }
}

void CZoomyClient::mat_to_tex(cv::Mat &input, GLuint &output) {
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
    if (argc != 3) {
        std::cerr << "Usage: client <host> <port>" << std::endl;
        return 1;
    }

    CZoomyClient c = CZoomyClient(cv::Size(854, 480), argv[1], argv[2]);
    c.run();
    return 0;
}