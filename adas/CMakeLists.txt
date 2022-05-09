## pcc & l2plus, two choose one

# set(IS_USE_PCC  OFF)

# if(IS_USE_PCC)
    # add_subdirectory(pcc)
# else()
    # add_subdirectory(l2plus)
# endif()


add_definitions(-DSL_DEBUG)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  $ENV{TBOX_CXXFLAGS} -Wall -Wextra -std=c++11")


include_directories("./include")


set(BIN "smlk_adas")

file(GLOB SRC  "src/*.cpp")

add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} pthread z smartlinksdk smlk-log tsp_service_api adas_shared dl)
install(TARGETS ${BIN} RUNTIME DESTINATION bin)

#copy test license file to /userdata/map/
# set(LICENSE "./inc/adas.license")
# install(FILES ${LICENSE} DESTINATION default/property/)
