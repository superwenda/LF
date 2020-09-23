# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.8)

project ("LFCameraCalib")

set(MICROVIEW_DIR "D:/lib/Microview/SDKAPI")
find_package(MICROVIEW)
include_directories(${MICROVIEW_INCLUDE_DIRS})

set(OpenCV_DIR "D:/lib/opencv/build")
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR})

# add_definitions(-DUNICODE -D_UNICODE)
#ADD_DEFINITIONS(-D_AFXDLL)#如果使用动态库的MFC，则加上_AFXDLL
add_definitions(/MT)
set(CMAKE_MFC_FLAG 1)

add_compile_options(/openmp)

# include_directories("D:/lib/LF/include")
# link_directories("D:/lib/LF/lib")

# 包含子项目。
add_subdirectory ("LFCameraCalib")
add_subdirectory ("LFDisparity")
add_subdirectory ("LFRendering")
add_subdirectory ("testapi")

add_library(common "common.cpp" "camera_gui.cpp")
target_link_libraries(common ${MICROVIEW_LIBS} ${OpenCV_LIBS})