
#add_compile_options("-O3")
add_compile_options("--std=c++11")

add_executable(LFRendering "LFRendering.cpp")
target_link_libraries(LFRendering  ${OpenCV_LIBS} ${MICROVIEW_LIBS})

#add_executable(LFRenderingCUDA "LFRenderingCUDA.cpp")
#target_link_libraries(LFRenderingCUDA  ${OpenCV_LIBS} ${MICROVIEW_LIBS})

add_executable(LFRenderingConcat "LFRenderingConcat.cpp")
target_link_libraries(LFRenderingConcat  ${OpenCV_LIBS} ${MICROVIEW_LIBS})