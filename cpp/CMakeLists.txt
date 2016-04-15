cmake_minimum_required (VERSION 2.6)

project (EMILY_functions)
include_directories(../arduino/libraries/ESP)
add_executable(esp ../testFunctions/esp_test.cpp)

add_executable(esp_parse ../testFunctions/esp_parse_bytes_test.cpp)

include_directories(../arduino/libraries/emilyStatus)
add_executable(status_test ../testFunctions/status_test.cpp ../arduino/libraries/emilyStatus/emilyStatus.cpp)

include_directories(../arduino/libraries/emilyGPS)
add_executable(convertGPS ../testFunctions/convertGPS_test.cpp ../arduino/libraries/emilyGPS/emilyGPS.cpp ../arduino/libraries/emilyStatus/emilyStatus.cpp)

add_executable(convertTime ../testFunctions/convertTime_test.cpp ../arduino/libraries/emilyGPS/emilyGPS.cpp ../arduino/libraries/emilyStatus/emilyStatus.cpp)

add_executable(testVenusCommands ../testFunctions/testVenusCommands.cpp ../arduino/libraries/emilyGPS/emilyGPS.cpp ../arduino/libraries/emilyStatus/emilyStatus.cpp)

include_directories(../arduino/libraries/controlPid ../arduino/libraries/lowPassFilter)
add_executable(pidTest ../testFunctions/pid_test.cpp ../arduino/libraries/controlPid/controlPid.cpp ../arduino/libraries/lowPassFilter/lowPassFilter.cpp)
