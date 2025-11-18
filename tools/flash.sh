rm -rf build
mkdir build
cd build
cmake .. -DNAVHAL=ON -DEXTERNAL_LINKER=ON
cmake --build . 
arm-none-eabi-objcopy -O binary main main.bin
st-flash --connect-under-reset write main.bin 0x8000000

