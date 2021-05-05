set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

set (sources
    sources/Robot.cpp
    sources/DCM.cpp
    sources/PID.cpp
    sources/Link.cpp

)
set(headers
    headers/Robot.h
    headers/DCM.h
    headers/PID.h
    headers/Link.h

)

add_cnoid_simple_controller(Surenaoffline sources/main.cpp ${sources}  ${headers})
