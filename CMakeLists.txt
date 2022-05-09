#add_definitions("-Werror")

add_subdirectory(ftp-upgrade)

add_subdirectory(fota)

#Data Acquisition (DAQ).
add_subdirectory(daq)

add_subdirectory(dyc)

if( $ENV{TBOX_PLATFORM} MATCHES "ubuntu")
else()
add_subdirectory(vehicle-ctrl)
add_subdirectory(loan_ctrl)
add_subdirectory(close-ctrl)
endif()

#GB17691
add_subdirectory(gb6)

#logger
add_subdirectory(logging)

#avm ctrl
add_subdirectory(avm-manager)

add_subdirectory(diag)

#基础版特有模块
if( $ENV{TBOX_VEHICLE_TYPE} MATCHES "base")
    #GB32960
    add_subdirectory(ev)
    #PCC/L2+
    add_subdirectory(adas)
else()

endif()
add_subdirectory(bt)


add_subdirectory(curl_test)
# add_subdirectory(ipc_call_test)
