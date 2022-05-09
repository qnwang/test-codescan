add_definitions(-DSL_DEBUG)

#add_definitions(-D_T_DEBUG)

add_definitions(-D_HJ_DEV)

# HJ 1239 过检
# add_definitions(-D_CERT_HJ_1239)

# GB 17691 过检
# add_definitions(-D_CERT_GB_17691)

add_subdirectory(simulator)

set(BIN "smlk_gb6")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fpermissive")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O2 -g3 -fpermissive")

include_directories("./include")
include_directories("./include/utility_17691/ini")
include_directories("./include/utility_17691/socket")

file(GLOB SRC  "src/*")
file(GLOB INI_SRC  "src/utility_17691/ini/*")
file(GLOB TCP_SRC  "src/utility_17691/socket/*")

add_executable(${BIN} ${SRC} ${INI_SRC} ${TCP_SRC})

target_link_libraries(${BIN} pthread sqlite3 PocoJSON PocoUtil PocoFoundation jsoncpp smlk-log obd_api vehicle_data_api async-client PocoFoundation smartlinksdk hsmapi)

install(TARGETS ${BIN} RUNTIME DESTINATION bin)
