include_directories("./include")
include_directories(${PROJECT_SOURCE_DIR}/framework/business/tsp/hsm/inc)

add_definitions(-Wno-unused-parameter -Wno-unused-variable)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  $ENV{TBOX_CFLAGS} -std=gnu99")
message(STATUS "CMAKE_C_FLAGS = ${CMAKE_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  $ENV{TBOX_CXXFLAGS} -Wall -Wextra")
message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")

set(BIN "ipc_call_test")

file(GLOB SRC  "./*.cpp")

add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} obd_api smlk_spi pthread)
if(WITH_HSM_ENCRYPT)
    target_link_libraries(${BIN} hsmapi)
endif(WITH_HSM_ENCRYPT)
install(TARGETS ${BIN} RUNTIME DESTINATION bin)
