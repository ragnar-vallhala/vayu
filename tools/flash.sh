rm -rf build
mkdir build
cd build
cmake .. -DVAIOS=ON -DSENSOR=ON
cmake --build .
arm-none-eabi-objcopy -O binary sensor/main main.bin
st-flash --connect-under-reset write main.bin 0x8000000
