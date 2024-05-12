# vendor/vika-net/CMakeLists.txt
message(STATUS "Fetching vika-net ...")

FetchContent_GetProperties(vika-net)
if (NOT vika-net_POPULATED)
    FetchContent_Populate(vika-net)
endif ()

add_library(vika-net
        ${vika-net_SOURCE_DIR}/src/CUDPClient.cpp
        ${vika-net_SOURCE_DIR}/src/CTCPClient.cpp
        ${vika-net_SOURCE_DIR}/src/CUDPServer.cpp
        ${vika-net_SOURCE_DIR}/include/CUDPClient.hpp
        ${vika-net_SOURCE_DIR}/include/CTCPClient.hpp
        ${vika-net_SOURCE_DIR}/include/CUDPServer.hpp
)

find_package(spdlog REQUIRED)

set_target_properties(vika-net PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(vika-net PUBLIC ${vika-net_SOURCE_DIR}/src ${vika-net_SOURCE_DIR}/include)
target_link_libraries(vika-net PUBLIC spdlog::spdlog)

FetchContent_MakeAvailable(vika-net)