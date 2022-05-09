set(BIN "GB17691_Platform")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories("./include")

file(GLOB SRC  "src/*")

add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} pthread)