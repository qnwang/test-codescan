include_directories("./include")
add_definitions( -Wno-unused-parameter)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  $ENV{TBOX_CFLAGS} -std=gnu99")
message(STATUS "CMAKE_C_FLAGS = ${CMAKE_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  $ENV{TBOX_CXXFLAGS} -Wall -Wextra")
message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

set(BIN "smlk_avm_ctrl")

file(GLOB SRC  "src/*.cpp")

add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} pthread PocoFoundation PocoNet tsp_service_api smartlinksdk vehicle_data_api)
install(TARGETS ${BIN} RUNTIME DESTINATION bin)
