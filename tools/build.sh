rm -rf build
mkdir build
cd build
cmake .. -DNAVHAL=ON -DEXTERNAL_LINKER=ON
cmake --build . 

