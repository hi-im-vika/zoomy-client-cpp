cmake_minimum_required(VERSION 3.10)
project(zoomy-client)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(OpenGL_GL_PREFERENCE LEGACY)

add_definitions(-DWINDOW_NAME="${CMAKE_PROJECT_NAME}")

add_executable(zoomy-client
        src/CZoomyClient.cpp
        src/CCommonBase.cpp
        src/CWindow.cpp
        src/CDPIHandler.cpp
        src/CAutoController.cpp
)

add_subdirectory(vendor)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE imgui vika-net)

set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

find_package(OpenGL REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE OpenGL::GL)

find_package(spdlog CONFIG REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE spdlog::spdlog_header_only)

find_package(SDL2 REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE SDL2::SDL2)

#find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)

if (WIN32)
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ws2_32 -lmingw32 -mwindows)
    add_definitions(-DSDL_MAIN_HANDLED)
endif ()