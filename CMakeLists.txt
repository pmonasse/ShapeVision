cmake_minimum_required(VERSION 3.13)
project(ShapeVision)

set(CMAKE_CXX_STANDARD 11)

find_package(PNG)
if(NOT PNG_FOUND)
  find_package(ZLIB) # zlib is needed by libPNG
  if(NOT ZLIB_FOUND)
        set(ZLIB_BUILD_EXAMPLES OFF)
        add_subdirectory(third_party/zlib-1.3.1)
        set(ZLIB_FOUND TRUE)
        set(PNG_BUILD_ZLIB ON)
        set_target_properties(${ZLIB_LIBRARIES} PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${ZLIB_INCLUDE_DIRS}")
        add_library(ZLIB::ZLIB ALIAS "${ZLIB_LIBRARIES}")
  endif()

  set(PNG_SHARED OFF)
  set(PNG_TESTS OFF)
  set(PNG_TOOLS OFF)
  set(SKIP_INSTALL_ALL 1)
  add_subdirectory(third_party/libpng-1.6.44)
  set_target_properties(${PNG_LIBRARIES} PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${PNG_INCLUDE_DIRS}")
  add_library(PNG::PNG ALIAS "${PNG_LIBRARIES}")
endif()

add_executable(shapeVision
    io_png.c io_png.h
    cmdLine.h
    cc.h cc.cpp
    shapeVision.cpp)

target_link_libraries(shapeVision PRIVATE PNG::PNG)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU)|(CLANG)")
  set_target_properties(shapeVision PROPERTIES COMPILE_FLAGS "-Wall -Wextra")
endif()

add_executable(testRect testRect.cpp cc.h cc.cpp)
