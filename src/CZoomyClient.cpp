/**
 * CZoomyClient.cpp - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CZoomyClient.hpp"

CZoomyClient::CZoomyClient(cv::Size s) {
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
    _video_capture.open(2);
    if (!_video_capture.isOpened()) {
        spdlog::error("Could not open cv");
        exit(-1);
    } else {
        spdlog::info("Camera opened with backend " + _video_capture.getBackendName());
        _video_capture.read(_img);
    }

    // preallocate texture handle

    glGenTextures(1, &_tex);
    glGenTextures(1, &_another_tex);
}

CZoomyClient::~CZoomyClient() = default;

void CZoomyClient::update() {
//    spdlog::info("Client update");
    _lockout.lock();
    _video_capture.read(_img);
    if (!_img.empty()) {
        _detector_params = cv::aruco::DetectorParameters();
        _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        _detector.setDetectorParameters(_detector_params);
        _detector.setDictionary(_dictionary);
        _detector.detectMarkers(_img, _marker_corners, _marker_ids, _rejected_candidates);
    }
    if (!_img.empty()) cv::aruco::drawDetectedMarkers(_img, _marker_corners, _marker_ids);
    _lockout.unlock();
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(10));
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

    ImGui::Begin("OpenCV", p_open);

    // from https://www.reddit.com/r/opengl/comments/114lxvr/imgui_viewport_texture_not_fitting_scaling_to/
    ImVec2 viewport_size = ImGui::GetContentRegionAvail();
    float ratio = ((float) _img.cols) / ((float) _img.rows);
    float viewport_ratio = viewport_size.x / viewport_size.y;

    _lockout.lock();
    mat_to_tex(_img, _tex);
    _lockout.unlock();

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
    ImGui::BeginGroup();
    ImGui::Button("Connect to AVFoundation");
    ImGui::SameLine();
    ImGui::Button("Connect to GStreamer");
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

int main() {
    CZoomyClient c = CZoomyClient(cv::Size(854, 480));
    c.run();
    return 0;
}