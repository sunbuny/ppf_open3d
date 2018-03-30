include_directories(/usr/include/eigen3/)
include_directories(/usr/include/GL)

set(OPEN3D_INCLUDE_DIRS /home/sun/src/Open3D/install/include)
link_directories(/home/sun/src/Open3D/install/lib)


find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(OPEN3D_LIBRARIES
        Visualization
        IO
        Core
        IO
        pthread
        tinyfiledialogs
        jpeg
        png
        jsoncpp
        zlib
        /usr/lib/x86_64-linux-gnu/libglfw.so
        /usr/lib/x86_64-linux-gnu/libGLEW.so
        /usr/lib/x86_64-linux-gnu/libdl.so
        /usr/lib/x86_64-linux-gnu/librt.so
        /usr/lib/x86_64-linux-gnu/libm.so
        /usr/lib/x86_64-linux-gnu/libX11.so
        /usr/lib/x86_64-linux-gnu/libXrandr.so
        /usr/lib/x86_64-linux-gnu/libXinerama.so
        /usr/lib/x86_64-linux-gnu/libXi.so
        /usr/lib/x86_64-linux-gnu/libXxf86vm.so
        /usr/lib/x86_64-linux-gnu/libXcursor.so
        /usr/lib/x86_64-linux-gnu/libGL.so
        )
