include_directories("./include")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  $ENV{TBOX_CFLAGS} -std=gnu99")
message(STATUS "CMAKE_C_FLAGS = ${CMAKE_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  $ENV{TBOX_CXXFLAGS} -Wall -Wextra")
message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

set(BIN "debug")

file(GLOB SRC  "*.cpp")
	
add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} smartlinksdk )

install(TARGETS ${BIN} RUNTIME DESTINATION bin)
