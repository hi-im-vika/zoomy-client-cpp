/**
 * CCommonBase.h - inheritable class with common functions
 * 2024-02-08
 * vika <https://github.com/hi-im-vika>
 */

#pragma once

#include <thread>

#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

/**
 * @brief A class designed to be inherited from to provide functions common to all
 * @author vika
 */
class CCommonBase {
public:

    /**
     * @brief Constructor for CCommonBase
     */
    CCommonBase() = default;

    /**
     * @brief Destructor for CCommonBase
     */
    ~CCommonBase();

    /**
     * @brief Pure virtual function for updating member variables. Must be defined in child objects.
     */
    virtual void update() = 0;

    /**
     * @brief Pure virtual function for drawing onto the canvas. Must be defined in child objects.
     */
    virtual void draw() = 0;

    /**
     * @brief Runs the update and draw functions.
     */
    void run();

    /**
     * @brief Multithreaded implementation of the update function
     * @param who_called_me The pointer to the CCommonBase object to update.
     */
    static void update_thread(CCommonBase *who_called_me);

    /**
     * @brief Multithreaded implementation of the draw function
     * @param who_called_me The pointer to the CCommonBase object to draw.
     */
    static void draw_thread(CCommonBase *who_called_me);

protected:
    cv::Size _window_size;      ///< The size of the window.
    std::chrono::steady_clock::time_point _perf_update_start;       ///< Start time for measuring performance.
    std::chrono::steady_clock::time_point _perf_draw_start;         ///< Start time for measuring performance.
    int _perf_update = 1;       ///< Measured update time in milliseconds.
    int _perf_draw = 1;         ///< Measured draw time in milliseconds.
    bool _do_exit = false;           ///< Flag to exit program.
};