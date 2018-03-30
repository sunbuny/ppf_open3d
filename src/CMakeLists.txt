find_package(Open3d)
include_directories(${OPEN3D_INCLUDE_DIRS})

set(SOURCES
        Params.cpp
        PPF.cpp
        PPF_PoseEstimation.cpp
        CPUTimer.cpp
        )

file(GLOB HEADERS
        ../include/*.h
        )

add_library(ppf
        ${SOURCES}
        ${HEADERS}
        )
target_link_libraries(ppf ${OPEN3D_LIBRARIES})


add_executable(main_test main.cpp)
target_link_libraries(main_test
        ppf
        ${OPEN3D_LIBRARIES}
#        Core
#        IO
        )