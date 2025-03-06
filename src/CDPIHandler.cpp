/**
 * CDPIHandler.cpp - new file
 * 2024-04-02
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CDPIHandler.hpp"

float CDPIHandler::get_scale() {

    constexpr int display_index = 0;
    // @todo: This should be 72.0F on Mac, but it seems like it is not. I'm not
    //  sure why, but this works ¯\_(ツ)_/¯
    const float default_dpi = 96.0F;
    float dpi = default_dpi;

    SDL_GetDisplayDPI(display_index, nullptr, &dpi, nullptr);
    spdlog::info("screens: {:d}", SDL_GetNumVideoDisplays());
    spdlog::info("dpi: {:.2f}",dpi);
    if ((dpi / default_dpi) < 1) {
        return 1;
    }
    return std::floor(dpi / default_dpi);
}

WindowSize CDPIHandler::get_dpi_aware_window_size(const config c) {

    return {c.width, c.height};
}

void CDPIHandler::set_global_font_scaling(ImGuiIO* io) {

    io->FontGlobalScale = 1.0F / get_scale();
}