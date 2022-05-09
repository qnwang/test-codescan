add_definitions(-DSL_DEBUG)

#if (MSVC)
#    # warning level 4 and all warnings as errors
#    add_compile_options(/W4 /WX)
#else()
#    # lots of warnings and all warnings as errors
#    add_compile_options(-Wall -Wextra -pedantic -Werror)
#endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  $ENV{TBOX_CXXFLAGS} -Wall -Wextra  ")


include_directories("./inc")


set(BIN "smlk_pcc")

file(GLOB SRC  "src/*.cpp")

add_executable(${BIN} ${SRC})

# target_link_libraries(${BIN} pthread z adasisv2hp smartlinksdk smlk-log tsp_service_api)
target_link_libraries(${BIN} pthread z smartlinksdk smlk-log tsp_service_api dl)
install(TARGETS ${BIN} RUNTIME DESTINATION bin)

#copy test license file to /userdata/map/
set(LICENSE "./inc/adas.license")
install(FILES ${LICENSE} DESTINATION default/property/)
