
add_executable(testapi "testapi.cpp")
target_link_libraries(testapi ${OpenCV_LIBS} ${OpenCV_LIBS} common)

add_executable(GrabImage "GrabImage.cpp")
target_link_libraries(GrabImage ${OpenCV_LIBS} ${MICROVIEW_LIBS})