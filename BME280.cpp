/*
 * BME280.cpp
 *
 *  Created on: 6/08/2021
 *      Author: Juan
 */
#include "BME280.h"

BME280::BME280(I2C_HandleTypeDef * i2c, const Settings& settings,std::uint8_t addr): m_i2c(i2c), m_settings(settings),
m_addr(addr){
	t_fine = 0;
#if DEBUG_BME
	for(int i = 0; i < DIG_LENGTH; i++){
		m_dig[i] = 0;
	}
#endif
}

bool BME280::Init(){
	bool success = true;

	if(GetID() != 0x60) return false;
	std::uint8_t reset[2] = {RESET_ADDR, 0xB6U};
	Write_reg(reset, 2);
	/*This is a custom function for delay, in case you are not using my custom libraries, please use HAL or your custom implementation here*/
	utils::delay::ms(10);
	/*-------------------------------------------------------------------------------------------------------------------------------------*/
	while(ReadingCalibration());


	success &= GetDig_T();

	dig_T1 = (m_dig[1] << 8) | m_dig[0];
	dig_T2 = (m_dig[3] << 8) | m_dig[2];
	dig_T3 = (m_dig[5] << 8) | m_dig[4];

	dig_H1 = m_dig[24];
	dig_H2 = (m_dig[26] << 8) | m_dig[25];
	dig_H3 = m_dig[27];
	dig_H4 = ((std::int8_t)m_dig[28] * 16) | (0x0F & m_dig[29]);
	dig_H5 = ((std::int8_t)m_dig[30] * 16) | ((m_dig[29] >> 4) & 0x0F);
	dig_H6 = m_dig[31];

	dig_P1 = (m_dig[7]  << 8) | m_dig[6];
	dig_P2 = (m_dig[9]  << 8) | m_dig[8];
	dig_P3 = (m_dig[11] << 8) | m_dig[10];
	dig_P4 = (m_dig[13] << 8) | m_dig[12];
	dig_P5 = (m_dig[15] << 8) | m_dig[14];
	dig_P6 = (m_dig[17] << 8) | m_dig[16];
	dig_P7 = (m_dig[19] << 8) | m_dig[18];
	dig_P8 = (m_dig[21] << 8) | m_dig[20];
	dig_P9 = (m_dig[23] << 8) | m_dig[22];


	/*Implement settings*/
	volatile std::uint8_t config_write = (((std::uint8_t)m_settings.m_sb << 5) | ((std::uint8_t)m_settings.m_filter << 2));

	volatile std::uint8_t ctrl_meas_write = (((std::uint8_t)m_settings.m_temp_osr << 5) | ((std::uint8_t)m_settings.m_press_osr << 2) |
											((std::uint8_t)m_settings.m_mode << 0));

	volatile std::uint8_t ctrl_hum = ((std::uint8_t)m_settings.m_hum_osr << 0);

	//std::uint8_t config_data[6] = {CONF_ADDR, config_write, CTRL_MEAS_ADDR, ctrl_meas_write, CTRL_HUM_ADDR, ctrl_hum};

	//success &= Write_reg(config_data, 6);
	std::uint8_t conf_data[2] = {CONF_ADDR, config_write};
	success &= Write_reg(conf_data, 2);
	std::uint8_t meas_data[2] = {CTRL_MEAS_ADDR, ctrl_meas_write};
	success &= Write_reg(meas_data, 2);
	std::uint8_t hum_data[2]  = {CTRL_HUM_ADDR, ctrl_hum};
	success &= Write_reg(hum_data, 2);



#if DEBUG_BME
	{
		std::uint8_t conf;
		std::uint8_t ctrl;
		std::uint8_t hum;

		Read_reg(&conf, CONF_ADDR,1);
		Read_reg(&ctrl, CTRL_MEAS_ADDR,1);
		Read_reg(&hum, CTRL_HUM_ADDR,1);

		GetID();
	}
#endif



	return success;
}

std::uint8_t BME280::GetID(){
	std::uint8_t id[1] = {0};
	Read_reg(id, ID_ADDR,1);
	return id[0];
}


/*private methods*/
bool BME280::GetDig_T(){
	std::uint8_t offset = 0;
	bool success = true;

	//read dig_t
	success &= Read_reg(&m_dig[offset], DIG_T1_ADDR,TEMP_DIG_LENGTH);
	offset += TEMP_DIG_LENGTH;


	//read press_t
	success &= Read_reg(&m_dig[offset], DIG_P1_ADDR,PRES_DIG_LENGTH);
	offset += PRES_DIG_LENGTH;

	//read hum_t (1)

	success &= Read_reg(&m_dig[offset], DIG_H1_ADDR,HUM1_DIG_LENGTH);
	offset += HUM1_DIG_LENGTH;

	//read hum_t (2)

	success &= Read_reg(&m_dig[offset], DIG_H2_ADDR,HUM2_DIG_LENGTH);
	offset += HUM2_DIG_LENGTH;

	return success && offset == DIG_LENGTH;

}


void BME280::ReadSensor(float& press, float& temp, float& hum){
	std::int8_t data[8];

	if(m_settings.m_mode == FORCED){
		volatile std::uint8_t config_write = (((std::uint8_t)m_settings.m_sb << 5) | ((std::uint8_t)m_settings.m_filter << 2));

		volatile std::uint8_t ctrl_meas_write = (((std::uint8_t)m_settings.m_temp_osr << 5) | ((std::uint8_t)m_settings.m_press_osr << 2) |
												((std::uint8_t)m_settings.m_mode << 0));

		volatile std::uint8_t ctrl_hum = ((std::uint8_t)m_settings.m_hum_osr << 0);
		std::uint8_t conf_data[2] = {CONF_ADDR, config_write};
		Write_reg(conf_data, 2);
		std::uint8_t meas_data[2] = {CTRL_MEAS_ADDR, ctrl_meas_write};
		Write_reg(meas_data, 2);
		std::uint8_t hum_data[2]  = {CTRL_HUM_ADDR, ctrl_hum};
		Write_reg(hum_data, 2);
	}
	if(!ReadSensorData(data)){
		press = temp = hum = 0;
		return;
	}

    std::uint32_t raw_press = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
	std::uint32_t raw_temp  = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
	std::uint32_t raw_hum   = (data[6] << 8)  | data[7];

	temp  = CalculateTemp(raw_temp);
	press = CalculatePress(raw_press);
	hum   = CalculateHum(raw_hum);


}

/*
 * Private functions
 * */

float BME280::CalculateTemp(std::int32_t raw){

	 std::int32_t var1, var2, final;


	 var1 = ((((raw >> 3) - ((std::int32_t)dig_T1 << 1))) * ((std::int32_t)dig_T2)) >> 11;
	 var2 = (((((raw >> 4) - ((std::int32_t)dig_T1)) * ((raw >> 4) - ((std::int32_t)dig_T1))) >> 12) * ((std::int32_t)dig_T3)) >> 14;
	 t_fine = var1 + var2;
	 final = (t_fine * 5 + 128) >> 8;

	 return final/100.0; // in °C
}

float BME280::CalculateHum(std::int32_t raw){
	   int32_t var1;


	   var1 = (t_fine - ((int32_t)76800));
	   var1 = (((((raw << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var1)) +
	   ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)dig_H6)) >> 10) * (((var1 *
	   ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	   ((int32_t)dig_H2) + 8192) >> 14));
	   var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	   var1 = (var1 < 0 ? 0 : var1);
	   var1 = (var1 > 419430400 ? 419430400 : var1);
	   return ((uint32_t)(var1 >> 12))/1024.0; //in %
}

float BME280::CalculatePress(std::int32_t raw){
	int64_t var1, var2, pressure;
	   float final;


	   var1 = (int64_t)t_fine - 128000;
	   var2 = var1 * var1 * (int64_t)dig_P6;
	   var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	   var2 = var2 + (((int64_t)dig_P4) << 35);
	   var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	   var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
	   if (var1 == 0) { return 0; }                                                         // Don't divide by zero.
	   pressure   = 1048576 - raw;
	   pressure = (((pressure << 31) - var2) * 3125)/var1;
	   var1 = (((int64_t)dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
	   var2 = (((int64_t)dig_P8) * pressure) >> 19;
	   pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

	   final = ((uint32_t)pressure)/256.0;

	   final /= 100.0; //in hPa

	   return final;
}

bool BME280::ReadingCalibration(void){
	std::uint8_t status = 0;
	Read_reg(&status, STATUS_ADDR, 1);

	return (status & (1 << 0)) != 0;
}

bool BME280::ReadSensorData(std::int8_t * data){
	bool success = true;
	std::uint8_t buffer[SENSOR_LENGTH];

	success &= Read_reg(buffer, PRESS_ADDR,SENSOR_LENGTH);
	for(int i = 0; i < SENSOR_LENGTH; i++){
		data[i] = static_cast<std::int32_t>(buffer[i]);
	}

	return success;
}

bool BME280::Write_reg(std::uint8_t data){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), &data , 1, HAL_MAX_DELAY) == HAL_OK);
}

bool BME280::Write_reg(std::uint8_t * data, std::uint8_t size){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), data , size, HAL_MAX_DELAY) == HAL_OK);
}

bool BME280::Read_reg(std::uint8_t * data, std::uint8_t mem_addr,std::uint8_t size){
	HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), &mem_addr , 1, HAL_MAX_DELAY);
	return (HAL_I2C_Master_Receive(m_i2c, (m_addr << 1), data, size, HAL_MAX_DELAY) == HAL_OK);
}

