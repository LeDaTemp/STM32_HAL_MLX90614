/*
 * mlx90614.c
 *
 *  Created on: Sep 13, 2023
 *      Author: fex95
 */


#include "mlx90614.h"

/**
  * @brief Initialize the MLX90614 sensor.
  * @param sensor_obj Pointer to the MLX90614 object.
  * @retval HAL status (indicates whether the sensor is ready to communicate).
  */
uint8_t MLX90614_init(MLX90614* sensor_obj){
	// Turn on power to the sensor to start it
	HAL_GPIO_WritePin(sensor_obj->power_gpio, sensor_obj->power_gpio_pin, GPIO_PIN_SET);
	HAL_Delay(200);

	// Calculate the internal I2C address
	sensor_obj->address_internal = sensor_obj->address << 1;

	// Check if the sensor is ready to communicate
	return HAL_I2C_IsDeviceReady(sensor_obj->interface, sensor_obj->address_internal, 3, 5);
}

/**
  * @brief Restart the MLX90614 sensor.
  * @param sensor_obj Pointer to the MLX90614 object.
  */
void MLX90614_restart(MLX90614* sensor_obj){
	// Turn off power to the sensor
	HAL_GPIO_WritePin(sensor_obj->power_gpio, sensor_obj->power_gpio_pin, GPIO_PIN_RESET);
	HAL_Delay(500);

	// Turn on power to the sensor to restart it
	HAL_GPIO_WritePin(sensor_obj->power_gpio, sensor_obj->power_gpio_pin, GPIO_PIN_SET);
	HAL_Delay(200);
}

/**
  * @brief Calculate CRC-8 checksum for data.
  * @param addr Pointer to the data buffer.
  * @param len Length of the data buffer.
  * @retval CRC-8 checksum value.
  */
uint8_t MLX90614_crc8(uint8_t *addr, uint8_t len){
	uint8_t crc = 0;
	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t carry = (crc ^ inbyte) & 0x80;
			crc <<= 1;
			if (carry)
				crc ^= 0x7;
			inbyte <<= 1;
		}
  }
  return crc;
}

/**
  * @brief Write a 16-bit value to the MLX90614's with PEC.
  * @param sensor_obj Pointer to the MLX90614 object.
  * @param reg Register address.
  * @param value 16-bit value to write.
  */
void MLX90614_writeWord_with_PEC(MLX90614* sensor_obj, uint8_t reg, uint16_t value){
	// Create a message with I2C address, register, value and add the PEC
	uint8_t msg[5];
	msg[0] = sensor_obj->address_internal;
	msg[1] = reg;
	msg[2] = value & 0xff;
	msg[3] = value >> 8;
	msg[4] = MLX90614_crc8(msg, 4);

	// Write the message
	HAL_I2C_Mem_Write(sensor_obj->interface, sensor_obj->address_internal, reg, 1, msg+2, 3, 10);
}

/**
  * @brief Read a 16-bit value from the MLX90614's EEPROM.
  * @param sensor_obj Pointer to the MLX90614 object.
  * @param reg Register address in the EEPROM.
  * @retval 16-bit value read from the EEPROM.
  */
uint16_t MLX90614_readEEPROM(MLX90614* sensor_obj, uint8_t reg){
	uint8_t result[2];
	HAL_I2C_Mem_Read(sensor_obj->interface, sensor_obj->address_internal, reg|0b00100000, 1, result, 2, 10);
	return (uint16_t)result[1]<<8|result[0];
}

/**
  * @brief Write a 16-bit value to the MLX90614's EEPROM with PEC and verify.
  * @param sensor_obj Pointer to the MLX90614 object.
  * @param reg Register address in the EEPROM.
  * @param value 16-bit value to write and verify.
  * @retval 0 if the write and verification succeed, -1 if they fail.
  */
uint8_t MLX90614_writeEEPROM(MLX90614* sensor_obj, uint8_t reg, uint16_t value){
	// Write a 16-bit value to the sensor's EEPROM with PEC
	MLX90614_writeWord_with_PEC(sensor_obj, reg|0b00100000, 0x0000);
	HAL_Delay(10);
	MLX90614_writeWord_with_PEC(sensor_obj, reg|0b00100000, value);
	HAL_Delay(10);

	// Check if the value was written correctly
	if(MLX90614_readEEPROM(sensor_obj, reg) != value)
		return -1;

	// If the address was written, update the sensor's address
	if(reg == MLX90614_EEPROM_I2C_ADDRESS){
		sensor_obj->address = value & 0xff;
		sensor_obj->address_internal = sensor_obj->address << 1;
	}

	// Restart the sensor
	MLX90614_restart(sensor_obj);
	return 0;
}

/**
  * @brief Read the ambient temperature from the MLX90614 sensor.
  * @param sensor_obj Pointer to the MLX90614 object.
  * @param data Pointer to a variable to store the temperature in degrees Celsius.
  */
void MLX90614_readAmbientTemperature(MLX90614* sensor_obj, float* data){
	// Read the ambient temperature from the sensor and convert it to degrees Celsius
	uint8_t result[2];
	HAL_I2C_Mem_Read(sensor_obj->interface, sensor_obj->address_internal, MLX90614_RAM_T_AMBIENT, 1, result, 2, 10);
	*data = ((uint16_t)result[1]<<8|result[0])*0.02f-273.15f;
}

