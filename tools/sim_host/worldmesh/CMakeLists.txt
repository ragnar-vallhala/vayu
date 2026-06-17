cmake_minimum_required(VERSION 3.16)
project(vsim_worldmesh CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui)
find_package(assimp REQUIRED)

set(_root ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

# Reuse the GCS's own mesh loader + BVH builder so the headless collision mesh
# is identical to what the Navigator ships to vsim_d.
add_executable(vsim_worldmesh
    vsim_worldmesh.cpp
    ${_root}/software/src/vsim/MeshLoader.cpp
    ${_root}/software/src/vsim/WorldMeshBuilder.cpp
)

target_include_directories(vsim_worldmesh PRIVATE
    ${_root}/software/src/vsim
    ${_root}/tools/vsim/include
)

target_link_libraries(vsim_worldmesh PRIVATE Qt6::Core Qt6::Gui assimp::assimp)
