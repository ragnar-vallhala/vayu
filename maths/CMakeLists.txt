# include_directories(${CMAKE_SOURCE_DIR}/include)
FILE(GLOB_RECURSE MATHS_SOURCE_FILES ./*.c)
message(STATUS "Maths sources: ${MATHS_SOURCE_FILES}")
add_library(maths ${MATHS_SOURCE_FILES})
target_link_libraries(maths PUBLIC hal -lgcc vaios -lm)

