# vendor/CMakeLists.txt

include(FetchContent)

FetchContent_Declare(
        cppbase
        GIT_REPOSITORY  "https://github.com/hi-im-vika/cppbase.git"
        GIT_TAG         origin/main
)

FetchContent_Declare(
        vika-net
        GIT_REPOSITORY  "https://github.com/hi-im-vika/vika-net.git"
        GIT_TAG         origin/main
)

add_subdirectory(cppbase)
add_subdirectory(vika-net)