cmake_minimum_required(VERSION 3.10)
project(zoomy-client)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(OpenGL_GL_PREFERENCE GLVND)

add_subdirectory(vendor)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
find_package(spdlog REQUIRED)

add_definitions(-DWINDOW_NAME="${CMAKE_PROJECT_NAME}")

add_executable(zoomy-client
        src/CZoomyClient.cpp
        include/CZoomyClient.hpp
        src/CCommonBase.cpp
        include/CCommonBase.hpp
        src/CWindow.cpp
        include/CWindow.hpp
        src/CDPIHandler.cpp
        include/CDPIHandler.hpp
        src/CAutoController.cpp
        include/CAutoController.hpp
)
target_link_libraries(zoomy-client ${OpenCV_LIBS} ${OPENGL_LIBRARY} imgui spdlog::spdlog SDL2::SDL2 vika-udp)