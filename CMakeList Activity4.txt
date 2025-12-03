# Sources set
LIST(APPEND SOURCES
    ../src/door_detector.cpp
    ../src/hungarian.cpp
    ../src/pointcloud_center_estimator.cpp
    ../src/qcustomplot.cpp
    ../src/ransac_line_detector.cpp
    ../src/room_detector.cpp
    ../src/specificworker.cpp
    ../src/time_series_plotter.cpp
)
# Headers set
LIST(APPEND HEADERS
    ../src/common_types.h
    ../src/door_detector.h
    ../src/hungarian.h
    ../src/image_processor.h
    ../src/munkres.h
    ../src/nominal_room.h
    ../src/pointcloud_center_estimator.h
    ../src/hungarian.h
    ../src/qcustomplot.h
    ../src/ransac_line_detector.h
    ../src/room_detector.h
    ../src/specificworker.h
    ../src/time_series_plotter.h
)
SET(LIBS ${LIBS} tbb Qt6PrintSupport)
LIST(APPEND LIBS ${LIBS})
INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)
