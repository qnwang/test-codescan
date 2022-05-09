include_directories("./include")
#include_directories("../../framework/services/business/tsp/api/include")



set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  $ENV{TBOX_CFLAGS} -std=gnu99")
message(STATUS "CMAKE_C_FLAGS = ${CMAKE_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  $ENV{TBOX_CXXFLAGS} -Wall -Wextra")
message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

set(BIN "smlk_vec_ctrl")

file(GLOB SRC  "src/*.cpp")

add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} pthread  sqlite3 PocoJSON PocoUtil PocoFoundation iconv tsp_service_api smlk_spi vehicle_data_api smartlinksdk signal_handle dl obd_api)
install(TARGETS ${BIN} RUNTIME DESTINATION bin)
