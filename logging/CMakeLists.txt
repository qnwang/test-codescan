include_directories("./include")
add_definitions(-Wno-unused-parameter -Wno-unused-variable)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  $ENV{TBOX_CFLAGS} -std=gnu99")
message(STATUS "CMAKE_C_FLAGS = ${CMAKE_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  $ENV{TBOX_CXXFLAGS} -Wall -Wextra")
message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

set(BIN "smlk_logger")

file(GLOB SRC  "src/*.cpp"
               "utils/*.cpp")

add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} pthread PocoJSON PocoUtil PocoFoundation PocoNet tsp_service_api smartlinksdk)
install(TARGETS ${BIN} RUNTIME DESTINATION bin)
