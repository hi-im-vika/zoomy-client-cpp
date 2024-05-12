# vendor/CMakeLists.txt

include(FetchContent)

FetchContent_Declare(
        imgui
        GIT_REPOSITORY "https://github.com/ocornut/imgui.git"
        GIT_TAG          origin/docking
)

FetchContent_Declare(
        vika-net
        GIT_REPOSITORY "https://github.com/hi-im-vika/vika-net.git"
        GIT_TAG         origin/main
)

add_subdirectory(imgui)
add_subdirectory(vika-net)