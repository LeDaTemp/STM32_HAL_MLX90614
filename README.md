# MLX90614 Sensor Library for STM32

This repository contains a C library for interfacing with the MLX90614 infrared temperature sensor on STM32 microcontrollers. The library provides functions to initialize the sensor, read and write data from/to the sensor's EEPROM, read temperature values, and more.

## Features

- Initialize and restart the MLX90614 sensor.
- Read and write data from/to the sensor's EEPROM with PEC (Packet Error Checking).
- Read ambient and object temperatures from the sensor.
- Convert temperature values from T-unit to degrees Celsius.

## File Structure

- `mlx90614.c`: The source file containing the library functions' implementations.
- `mlx90614.h`: The header file defining the library's functions and data structures.
- `main.c`: An example usage of the library to interface with the MLX90614 sensor.

## Connect and configure

- connect the sensor via I2C and set the interface in the device configuration tool also to I2C (not SMBUS)
- Connect VCC of the MLX90614 direct to a GPIO of the STM32. This is needed to perform a reset after writing the EEPROM. (The GPIO is automatically switched on during intialization)
- Configure the GPIO as output in the device configurator tool

## Getting Started

To use this library in your STM32 project, follow these steps:

1. Include the `mlx90614.c` and `mlx90614.h` files in your project.
2. Include the necessary STM32 HAL in `mlx90614.h`.
3. Create an instance of the `MLX90614` structure and configure it with your specific settings, such as I2C interface and GPIO pin which is used as vcc of the sensor.
4. Use the library functions to interact with the MLX90614 sensor.

## Example

**MLX - STM32 Connections**
- VCC - PC6
- SDA - PB9
- SCL - PB8
- GND - GND

```c
...
#include "mlx90614.h"
...
int main(void){
 ...
MLX90614 sensor[1];
sensor->address = 0x5A;
sensor->interface = &hi2c1;
sensor->power_gpio = GPIOC;
sensor->power_gpio_pin = GPIO_PIN_6;

if(MLX90614_init(sensor) != HAL_OK) printf("Can't connect!\r\n");


MLX90614_writeEEPROM(sensor, MLX90614_EEPROM_I2C_ADDRESS, 0x005A);

int newAddress = MLX90614_readEEPROM(sensor, MLX90614_EEPROM_I2C_ADDRESS);
printf("New address: %X\r\n", newAddress);
 
 /* Infinite loop */
 while (1) {
   float t_a, t_obj;
   MLX90614_readAmbientTemperature(sensor, &t_a);
   MLX90614_readObjTemperature(sensor, &t_obj, MLX90614_OBJ1);
   printf("t_a: %f\t t_obj: %f\r\n", t_a, t_obj);
   HAL_Delay(500);
 }
...
}
...
```
