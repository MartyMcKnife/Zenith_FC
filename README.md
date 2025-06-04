
# Zenith Flight Computer

This project was created to make a small scale hobby rocket flight computer, that can:
- Measure altitude
- Measure acceleration 
- Measure tilt
- Monitor battery voltage / charge
- Fit in a 38mm body tube
- Store flight performance on flash storage
- Communicate with PC over USB

## Component Selection

### Flight Computer
- __Selected MCU__: STM32G0B1CBT6
_Why?_
- Want to experiment with STM32 MCUs
- Low cost (cheapest on LCSC)
- Contains 2 i2C lines and 2 SPI lines - more than enough for peripherals
- USB-C built in - can do serial debugging and uploading all through USB-C

### Barometer
- __Selected Barometer__: MS5607
- Interface: SPI (SPI2)
_Why?_
- Flight tested
- Used before - very small package
### IMU
- __Selected__: LSM6DSO32
- Interface: I2C (I2C2)
_Why?_
- Up to 32gs accel reading - important for quickly accelerating rockets!
- User programmable interrupts - allows accel to detect launch, not MCU
- 9kB buffer
Note that due to the high cost ($7 per IC), a simpler LSM6DS3TR is used. This is a drop in replacement module, and only costs 40c per IC. The only disadvantage is it can read up to 16g - this may peak and cut-off during launch. Once the design is validated, it can be replaced with this IC

### Storage
- __Selected__: W25Q64
- Interface: SPI (SPI1)
_Why?_
- Very fast read/write
- Very proved hardware
- High capacity - 64 megabits
### Fuel Gauge
- __Selected__: BQ27441DRZR-G1B
- Interface: I2C (I2C2)
_Why?_
- Very simple to set up, minimal overhead
- High accuracy tracking
- DOES NOT CHARGE. Only measures voltage
