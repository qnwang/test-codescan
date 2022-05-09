add_definitions(-DSL_DEBUG)

include_directories("./include")

set(BIN "smlk_diag")

file(GLOB SRC  "src/*.cpp")

add_executable(${BIN} ${SRC})

target_link_libraries(${BIN} PocoData PocoDataSQLite sqlite3 PocoJSON PocoUtil PocoFoundation pthread smlk-log rxd-sdk smlk_spi tsp_service_api smartlinksdk smlk-ipc)

install(TARGETS ${BIN} RUNTIME DESTINATION bin)
