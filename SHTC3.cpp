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

bool SHTC3::begin(){
	/*wake up device*/
	wake_up();
	/*read device*/

	if(Read_ID() != 0x0807){
		return false;
	}

	return true;
}

std::uint16_t SHTC3::Read_ID(void){
	std::uint8_t data[2] = {0};
	if(!Read_reg(data, ID_CMD, 2, 2)){
		return 0;
	}
	std::uint16_t res = data[0] << 8 | data[1];
	std::uint16_t id  = res & SHTC3_PRODUCT_MASK;

	return id;

}

void SHTC3::wake_up(){
	Write_reg16(SHTC3_WAKE_UP_CMD);
	utils::delay::us(250);
}

void SHTC3::sleep(){
	Write_reg16(SHTC3_SLEEP_CMD);
}

bool SHTC3::Write_reg16(std::uint16_t data){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), (std::uint8_t *)&data , 2, HAL_MAX_DELAY) == HAL_OK);
}

bool SHTC3::Write_reg(std::uint8_t * data, std::uint8_t size){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), data , size, HAL_MAX_DELAY) == HAL_OK);
}

bool SHTC3::Read_reg(std::uint8_t * data, std::uint16_t mem_addr ,std::uint8_t size_t, std::uint8_t size_r){
	HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), (std::uint8_t *)&mem_addr , size_t, HAL_MAX_DELAY);
	return (HAL_I2C_Master_Receive(m_i2c, (m_addr << 1), data, size_r, HAL_MAX_DELAY) == HAL_OK);
}


