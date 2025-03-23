/**
 * CDPIHandler.h - DPI handler from cpp temnplate
 * 2024-04-02
 * vika <https://github.com/hi-im-vika>
 */

#pragma once

#include <cmath>

#include <SDL2/SDL.h>
#include <imgui.h>

#include "CWindow.hpp"

struct WindowSize {
    int width;
    int height;
};

class CDPIHandler {
public:
    [[nodiscard]] static float get_scale();

    [[nodiscard]] static WindowSize get_dpi_aware_window_size(const config c);

    static void set_global_font_scaling(ImGuiIO *io);


};