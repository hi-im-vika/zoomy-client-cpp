/**
 * CWindow.cpp - new file
 * 2024-03-29
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CWindow.hpp"

CWindow::CWindow(const std::string& title, const int width, const int height) {
    _window_config.title = title;
    _window_config.width = width;
    _window_config.height = height;

    // Create window with graphics context
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    auto window_flags = (SDL_WindowFlags) (SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    _window = SDL_CreateWindow(title.c_str(),width,height,window_flags);

    // NOLINTNEXTLINE
    _gl_context = SDL_GL_CreateContext(_window);
    if (_gl_context == nullptr) {
        spdlog::error("Could not create SDL OpenGL context.");
        return;
    }

    SDL_GL_MakeCurrent(_window, _gl_context);
    SDL_GL_SetSwapInterval(1);  // Enable vsync
}

CWindow::~CWindow() {
    SDL_GL_DestroyContext(_gl_context);
    SDL_DestroyWindow(_window);
}

SDL_Window *CWindow::get_native_window() const {
    return _window;
}

SDL_GLContext CWindow::get_native_context() const {
    return _gl_context;
}

config CWindow::get_config() const {
    return _window_config;
}