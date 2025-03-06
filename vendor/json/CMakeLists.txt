# vendor/json/CMakeLists.txt
message(STATUS "Fetching json ...")

FetchContent_GetProperties(json)
if (NOT json_POPULATED)
    FetchContent_MakeAvailable(json)
endif ()

FetchContent_MakeAvailable(json)