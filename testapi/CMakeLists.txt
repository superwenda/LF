
add_executable(testapi "testapi.cpp")
target_link_libraries(testapi ${OpenCV_LIBS} ${OpenCV_LIBS} common)

add_executable(GrabImage "GrabImage.cpp")
target_link_libraries(GrabImage ${OpenCV_LIBS} ${MICROVIEW_LIBS})

add_executable(MatTest "MatTest.cpp")
target_link_libraries(MatTest ${OpenCV_LIBS})

add_executable(RenderCenterTest "RenderCenterTest.cpp")
target_link_libraries(RenderCenterTest ${OpenCV_LIBS})