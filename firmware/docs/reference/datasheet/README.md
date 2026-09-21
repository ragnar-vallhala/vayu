# Vendor datasheets

The datasheets for this board's parts are **not stored here**. They are the
vendors' copyrighted documents, and this repository is public, so mirroring
them would be redistributing them outside their terms. Download them from the
source instead — they are free, and you get the current revision rather than
whatever was captured years ago.

| Part | Document | Where |
|---|---|---|
| Bosch BMX160 | 9-axis IMU datasheet | [bosch-sensortec.com](https://www.bosch-sensortec.com/products/motion-sensors/absolute-orientation-sensors/bmx160/) |
| STM32F401RE | datasheet (DS10086) | [st.com/en/microcontrollers-microprocessors/stm32f401re.html](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html) |
| STM32F401xE | reference manual (RM0368) | [st.com](https://www.st.com/resource/en/reference_manual/rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) |
| NUCLEO-F401RE | board user manual (UM1724) | [st.com/en/evaluation-tools/nucleo-f401re.html](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) |
| Bosch BME280 | environmental sensor | [bosch-sensortec.com](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/) |
| ST VL53L0X | time-of-flight ranger | [st.com/en/imaging-and-photonics-solutions/vl53l0x.html](https://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html) |

Register-level details this firmware actually depends on are recorded in the
driver sources and in the notes under `reference/imu/`, so the code is readable
without the PDFs to hand.
