add_definitions(-DSL_DEBUG)

#if (MSVC)
#    # warning level 4 and all warnings as errors
#    add_compile_options(/W4 /WX)
#else()
#    # lots of warnings and all warnings as errors
#    add_compile_options(-Wall -Wextra -pedantic -Werror)
#endif()

include_directories("./inc")
include_directories("../pcc/inc")


set(BIN "smlk_l2plus")

file(GLOB SRC  "src/*.cpp")

add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} pthread z smlk-log adas_shared smartlinksdk)

install(TARGETS ${BIN} RUNTIME DESTINATION bin)

#copy test license file to /userdata/map/
set(LICENSE "./inc/adas.license")
install(FILES ${LICENSE} DESTINATION default/property/)

set(l2Ini "./inc/adas.ini")
install(FILES ${l2Ini} DESTINATION default/property/)