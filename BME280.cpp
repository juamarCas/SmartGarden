/*
 * BME280.cpp
 *
 *  Created on: 6/08/2021
 *      Author: Juan
 */
#include "BME280.h"

BME280::BME280(I2C_HandleTypeDef * i2c, const Settings& settings,std::uint8_t addr): m_i2c(i2c), m_settings(settings),
m_addr(addr){
#if DEBUG_BME
	for(int i = 0; i < DIG_LENGTH; i++){
		m_dig[i] = 0;
	}
#endif
}

bool BME280::Init(){
	bool success = true;

	success &= GetDig_T();

	/*Implement settings*/
	volatile std::uint8_t config_write = (((std::uint8_t)m_settings.m_sb << 5) | ((std::uint8_t)m_settings.m_filter << 2));

	volatile std::uint8_t ctrl_meas_write = (((std::uint8_t)m_settings.m_temp_osr << 5) | ((std::uint8_t)m_settings.m_press_osr << 2) |
									((std::uint8_t)m_settings.m_mode << 0));

	volatile std::uint8_t ctrl_hum = ((std::uint8_t)m_settings.m_hum_osr << 0);

	std::uint8_t config_data[6] = {CONF_ADDR, config_write, CTRL_MEAS_ADDR, ctrl_meas_write, CTRL_HUM_ADDR, ctrl_hum};

	Write_reg(config_data, 6);


	return success;
}

std::uint8_t BME280::GetID(){
	std::uint8_t id[1] = {0};
	Write_reg(ID_ADDR);
	Read_reg(id, 1);
	return id[0];
}


/*private methods*/
bool BME280::GetDig_T(){
	std::uint8_t offset = 0;
	bool success = true;

	//read dig_t
	success &= Write_reg(DIG_T1_ADDR);
	success &= Read_reg(&m_dig[offset], TEMP_DIG_LENGTH);
	offset += TEMP_DIG_LENGTH;

	//read press_t
	success &= Write_reg(DIG_P1_ADDR);
	success &= Read_reg(&m_dig[offset], PRES_DIG_LENGTH);
	offset += PRES_DIG_LENGTH;

	//read hum_t (1)
	success &= Write_reg(DIG_H1_ADDR);
	success &= Read_reg(&m_dig[offset], HUM1_DIG_LENGTH);
	offset += HUM1_DIG_LENGTH;

	//read hum_t (2)
	success &= Write_reg(DIG_H2_ADDR);
	success &= Read_reg(&m_dig[offset], HUM2_DIG_LENGTH);
	offset += HUM2_DIG_LENGTH;

	return success && offset == DIG_LENGTH;

}



/*
 * Writing and reading examples of the I2C in the Datasheet of the module page 33 of the PDF
 * Datasheet: https://www.mouser.com/datasheet/2/783/BST-BME280-DS002-1509607.pdf
 * */
bool BME280::Write_reg(std::uint8_t data){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), &data , 1, HAL_MAX_DELAY) == HAL_OK);
}

bool BME280::Write_reg(std::uint8_t * data, std::uint8_t size){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), data , size, HAL_MAX_DELAY) == HAL_OK);
}

bool BME280::Read_reg(std::uint8_t * data, std::uint8_t size){
	return (HAL_I2C_Master_Receive(m_i2c, (m_addr << 1), data, size, HAL_MAX_DELAY) == HAL_OK);
}

