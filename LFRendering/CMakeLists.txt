
#add_compile_options("-O3")

add_library(render "render.cpp")
target_link_libraries(render ${OpenCV_LIBS})

add_executable(LFRender "LFRender.cpp")
target_link_libraries(LFRender  ${OpenCV_LIBS} ${MICROVIEW_LIBS} common render)

add_executable(RenderTest "RenderTest.cpp")
target_link_libraries(RenderTest render ${OpenCV_LIBS})

add_executable(LFRendering "LFRendering.cpp")
target_link_libraries(LFRendering  ${OpenCV_LIBS} ${MICROVIEW_LIBS})

#add_executable(LFRenderingCUDA "LFRenderingCUDA.cpp")
#target_link_libraries(LFRenderingCUDA  ${OpenCV_LIBS} ${MICROVIEW_LIBS})

add_executable(LFRenderingConcat "LFRenderingConcat.cpp")
target_link_libraries(LFRenderingConcat  ${OpenCV_LIBS} ${MICROVIEW_LIBS})