/**
 * CWindow.h - new file
 * 2024-03-29
 * vika <https://github.com/hi-im-vika>
 */

#pragma once

#include <string>

#include <spdlog/spdlog.h>
#include <SDL3/SDL.h>

struct config {
    std::string title;
    int width = 1280;
    int height = 720;
};

class CWindow {
private:
    SDL_Window *_window;
    SDL_GLContext _gl_context;
    config _window_config;

public:
    CWindow(const std::string& title, const int width, const int height);
    ~CWindow();

    [[nodiscard]] SDL_Window *get_native_window() const;
    [[nodiscard]] SDL_GLContext get_native_context() const;

    config get_config() const;
};