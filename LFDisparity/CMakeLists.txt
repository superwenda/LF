
add_executable(LFDisparity "LFDisparity.cpp")
target_link_libraries (LFDisparity ${OpenCV_LIBS})

add_executable(LFDeepth "LFDeepth.cpp")
target_link_libraries (LFDeepth ${OpenCV_LIBS})

add_executable(CVDisparity "CVDisparity.cpp")
target_link_libraries (CVDisparity ${OpenCV_LIBS})