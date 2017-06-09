# bump version here
set(slpk_VERSION 1.0)

define_module(LIBRARY slpk=${slpk_VERSION}
  DEPENDS roarchive>=1.1 geo>=1.24 imgproc>=1.20 geometry>=1.8
  math>=1.2 utility>=1.11 dbglog>=1.4
  jsoncpp>=1.1
  Boost_FILESYSTEM
  JPEG # need to measure JPEG images
  )

set(slpk_SOURCES
  reader.hpp reader.cpp
)

add_library(slpk STATIC ${slpk_SOURCES})
buildsys_library(slpk)

target_link_libraries(slpk ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(slpk ${MODULE_DEFINITIONS})
