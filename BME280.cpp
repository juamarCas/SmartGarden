/*
 * BME280.cpp
 *
 *  Created on: 6/08/2021
 *      Author: Juan
 */
#include "BME280.h"

BME280::BME280(I2C_HandleTypeDef * i2c, const Settings& settings,std::uint8_t addr): m_i2c(i2c), m_settings(settings),
m_addr(addr){}

bool BME280::Init(){
	if(!(GetID() & 0xFF)){
		return false;
	}

	return true;
}

std::uint8_t BME280::GetID(){
	std::uint8_t id[1] = {0};
	Write_reg(ID_ADDR);
	Read_reg(id, 1);
	return id[0];
}


/*private methods*/

void BME280::DIG_T(){

}

void BME280::DIG_P(){

}

void BME280::DIG_H1(){

}

void BME280::DIG_H2(){

}



/*
 * Writing and reading examples of the I2C in the Datasheet of the module page 33 of the PDF
 * Datasheet: https://www.mouser.com/datasheet/2/783/BST-BME280-DS002-1509607.pdf
 * */
void BME280::Write_reg(std::uint8_t data){
	HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), &data , 1, HAL_MAX_DELAY);
}

void BME280::Read_reg(std::uint8_t * data, std::uint8_t size){
	 HAL_I2C_Master_Receive(m_i2c, (m_addr << 1), data, size, HAL_MAX_DELAY);
}

