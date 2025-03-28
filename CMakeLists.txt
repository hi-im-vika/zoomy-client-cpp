set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

cmake_minimum_required(VERSION 3.10)
project(zoomy-client)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DWINDOW_NAME="${CMAKE_PROJECT_NAME}")

add_executable(zoomy-client
        src/CZoomyClient.cpp
        src/CWindow.cpp
        src/CDPIHandler.cpp
        src/CAutoController.cpp
)

add_subdirectory(vendor)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE cppbase vika-net)

find_package(imgui CONFIG REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE imgui::imgui)

find_package(OpenCV CONFIG REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

find_package(OpenGL REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE OpenGL::GL)

find_package(spdlog CONFIG REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE spdlog::spdlog_header_only)

find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)

if (WIN32)
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ws2_32 -lmingw32 -mwindows)
    add_definitions(-DSDL_MAIN_HANDLED)
endif ()