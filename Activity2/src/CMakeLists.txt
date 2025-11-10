# Sources set
LIST(APPEND SOURCES
    ../src/specificworker.cpp
    ../src/hungarian.cpp
    ../src/ransac_line_detector.cpp
    ../src/room_detector.cpp
)
# Headers set
LIST(APPEND HEADERS
    ../src/specificworker.h
    ../src/hungarian.h
    ../src/ransac_line_detector.h
    ../src/room_detector.h
)
SET(LIBS ${LIBS} tbb)
LIST(APPEND LIBS ${LIBS})
INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)
