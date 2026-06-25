cmake_minimum_required(VERSION 3.16)
project(vsim_worldmesh CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui)
find_package(assimp REQUIRED)

# repo root is 4 levels up: cpp/worldmesh -> cpp -> headless-sdk -> navigator -> root
set(_root ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)

# Reuse the GCS's own mesh loader + BVH builder so the headless collision mesh
# is identical to what the Navigator ships to vsim_d.
add_executable(vsim_worldmesh
    vsim_worldmesh.cpp
    ${_root}/navigator/src/vsim/MeshLoader.cpp
    ${_root}/navigator/src/vsim/WorldMeshBuilder.cpp
)

target_include_directories(vsim_worldmesh PRIVATE
    ${_root}/navigator/src/vsim
    ${_root}/sim/vsim/include
)

target_link_libraries(vsim_worldmesh PRIVATE Qt6::Core Qt6::Gui assimp::assimp)
