# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
)
# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
)
SET(LIBS ${LIBS} tbb)
INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)
