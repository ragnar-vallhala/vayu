
# CMSIS DSP for maths
set(CMSISCORE ${CMAKE_CURRENT_SOURCE_DIR}/../extern/CMSIS_5/CMSIS/Core)
set(CMSISDSP ${CMAKE_CURRENT_SOURCE_DIR}/../extern/CMSIS-DSP)
add_subdirectory(${CMSISDSP}/Source bin_dsp)
# include_directories(${CMSISDSP}/Include 
# ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS "${CMSISDSP}/Include")
FILE(GLOB_RECURSE SOURCE_FILES ./*.c)
message(STATUS "Maths sources: ${SOURCE_FILES}")
add_library(nav_maths ${SOURCE_FILES})
target_include_directories(nav_maths PRIVATE 
  ${CMSISDSP}/Include
  ${CMSISCORE}/Include
)
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "cortex-m4")
target_compile_definitions(nav_maths PRIVATE
    ARM_MATH_CM4
    __FPU_PRESENT=1
)
else()
  message(FATAL_ERROR "Unsupported CPU type ${CMAKE_SYSTEM_PROCESSOR}")
endif()
target_link_libraries(nav_maths PUBLIC hal -lgcc vaios CMSISDSP)

