# vendor/CMakeLists.txt

include(FetchContent)

FetchContent_Declare(
        vika-net
        GIT_REPOSITORY  "https://github.com/hi-im-vika/vika-net.git"
        GIT_TAG         origin/main
)

add_subdirectory(vika-net)