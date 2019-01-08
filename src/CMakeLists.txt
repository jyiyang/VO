add_executable(VO main.cpp odometry.cpp visualization.cpp mapping.cpp)
target_link_libraries(VO
    ${COLMAP_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
)