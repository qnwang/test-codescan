add_definitions(-DSL_DEBUG)

#if (MSVC)
#    # warning level 4 and all warnings as errors
#    add_compile_options(/W4 /WX)
#else()
#    # lots of warnings and all warnings as errors
#    add_compile_options(-Wall -Wextra -pedantic -Werror)
#endif()

include_directories("./inc")

if( $ENV{TBOX_PLATFORM}  MATCHES  "ubuntu")
    add_definitions(-DPLATFORM_X86)
else()
    add_definitions(-DPLATFORM_ARM)
endif()

set(BIN "smlk_dyc")

file(GLOB SRC  "src/*.cpp")

add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} pthread z sqlite3 jsoncpp smlk-log smlk-ipc curl tsp_service_api PocoFoundation PocoNet vehicle_data_api smartlinksdk signal_handle)

install(TARGETS ${BIN} RUNTIME DESTINATION bin)
