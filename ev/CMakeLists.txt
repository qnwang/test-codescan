add_definitions(-DSL_DEBUG)

add_definitions(-D_T_DEBUG)

set(BIN "smlk_ev")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -fpermissive")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O2 -g3 -pthread  -fpermissive")


include_directories(
        ./include
        ./include/ThreadPoll
        ./include/utility_32960
        ./include/utility_32960/ini
        ./include/utility_32960/timer
        ./include/utility_32960/socket
        ./include/message_32960
)


file(GLOB SRC "src/*")
file(GLOB THR_SRC "src/ThreadPoll/*")
file(GLOB UTI_SRC "src/utility_32960/*")
file(GLOB INI_SRC "src/utility_32960/ini/*")
file(GLOB TIM_SRC "src/utility_32960/timer/*")
file(GLOB TCP_SRC "src/utility_32960/socket/*")
file(GLOB MSG_SRC "src/message_32960/*")


add_executable(${BIN} ${SRC} ${THR_SRC} ${UTI_SRC} ${INI_SRC} ${TIM_SRC} ${TCP_SRC} ${MSG_SRC})

target_link_libraries(${BIN} pthread PocoData PocoDataSQLite sqlite3 jsoncpp smlk-log obd_api vehicle_data_api PocoFoundation smartlinksdk)

install(TARGETS ${BIN} RUNTIME DESTINATION bin)