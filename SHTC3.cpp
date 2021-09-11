/*
 * SHTC3.cpp
 *
 *  Created on: Sep 4, 2021
 *      Author: yiond
 */

#include "SHTC3.h"

SHTC3::SHTC3(I2C_HandleTypeDef * i2c, std::uint8_t addr) {
	m_addr = addr;
	m_i2c = i2c;
}

std::uint16_t SHTC3::Read_ID(void){
	std::uint8_t data[2] = {0,0};

}


bool SHTC3::Write_reg(std::uint8_t data){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), &data , 1, HAL_MAX_DELAY) == HAL_OK);
}

bool SHTC3::Write_reg(std::uint8_t * data, std::uint8_t size){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), data , size, HAL_MAX_DELAY) == HAL_OK);
}

bool SHTC3::Read_reg(std::uint8_t * data, std::uint8_t mem_addr,std::uint8_t size){
	HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), &mem_addr , 1, HAL_MAX_DELAY);
	return (HAL_I2C_Master_Receive(m_i2c, (m_addr << 1), data, size, HAL_MAX_DELAY) == HAL_OK);
}

bool SHTC3::Read_reg16(std::uint8_t * data, std::uint8_t * mem_addr){
	HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), &mem_addr[0] , 2, HAL_MAX_DELAY);
}

