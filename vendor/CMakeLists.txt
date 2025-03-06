# vendor/CMakeLists.txt

include(FetchContent)

FetchContent_Declare(
        imgui
        GIT_REPOSITORY  "https://github.com/ocornut/imgui.git"
        GIT_TAG         origin/docking
)

FetchContent_Declare(
        json
        URL             "https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz"
)


FetchContent_Declare(
        vika-net
        GIT_REPOSITORY  "https://github.com/hi-im-vika/vika-net.git"
        GIT_TAG         origin/main
)

add_subdirectory(imgui)
add_subdirectory(json)
add_subdirectory(vika-net)