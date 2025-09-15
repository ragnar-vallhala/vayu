# include_directories(${CMAKE_SOURCE_DIR}/include)
FILE(GLOB_RECURSE SOURCE_FILES ./*.c)
message(STATUS "Driver sources: ${SOURCE_FILES}")
add_library(drivers ${SOURCE_FILES})
target_link_libraries(drivers PUBLIC hal -lgcc vaios)

