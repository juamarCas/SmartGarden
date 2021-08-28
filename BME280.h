/*
 * BME280.h
 *
 *  Created on: 6/08/2021
 *      Author: Juan
 *  Datasheet: https://www.mouser.com/datasheet/2/783/BST-BME280-DS002-1509607.pdf
 *  i got some code from this git repository: https://github.com/finitespace/BME280/blob/master/src/BME280.h
 */

#ifndef BME280_H_
#define BME280_H_

/*change this for the stm you are using*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include "utils.h"
#include <cstdint>

#ifndef DEBUG_BME
	#define DEBUG_BME 1
#endif

class BME280{
public:

	enum Mode{
		SLEEP  = 0x0,
		FORCED = 0x1,
		NORMAL = 0x3
	};

	enum OSR{
		OSR_SKIPP = 0,
		OSR_X1    = 1,
		OSR_X2    = 2,
		OSR_x4    = 3,
		OSR_X8    = 4,
		OSR_X16   = 5
 	};

	enum Filter{
		FILTER_OFF    = 0,
		FILTER_COF_2  = 1,
		FILTER_COF_4  = 2,
		FILTER_COF_8  = 3,
		FILTER_COF_16 = 4
	};

	enum t_standby{
		T_SB_0d5   = 0,
		T_SB_62d5  = 1,
		T_SB_125   = 2,
		T_SB_250   = 3,
		T_SB_500   = 4,
		T_SB_1000  = 5,
		T_SB_10    = 6,
		T_SB_20    = 7
	};

	struct Settings{
		Settings(
				 Mode _mode      = FORCED,
				 Filter _filter  = FILTER_OFF,
				 OSR _temp_osr   = OSR_X1,
				 OSR _hum_osr    = OSR_X1,
				 OSR _press_osr  = OSR_X1,
				 t_standby _sb   = T_SB_1000
				): m_mode(_mode),
				   m_filter(_filter),
				   m_temp_osr(_temp_osr),
				   m_hum_osr(_hum_osr),
				   m_press_osr(_press_osr),
				   m_sb(_sb){}
		Mode m_mode;
		Filter m_filter;
		OSR  m_temp_osr;
		OSR  m_hum_osr;
		OSR  m_press_osr;
		t_standby m_sb;
	};



	BME280(I2C_HandleTypeDef * i2c, const Settings& settings = Settings(), std::uint8_t addr = 0x76U);
	bool Init();
	bool SetSettings();
	std::uint8_t GetID();

	float GetTemperature();
	float GetHumidity();
	float GetPressure();

	void ReadSensor(float& press, float& temp, float& hum);


private:
	I2C_HandleTypeDef * m_i2c;

	std::uint8_t m_dig[32];
	std::uint8_t m_addr;

	const std::uint8_t ID_ADDR        = 0xD0U;
	const std::uint8_t CONF_ADDR      = 0xF5U;
	const std::uint8_t CTRL_MEAS_ADDR = 0xF4U;
	const std::uint8_t CTRL_HUM_ADDR  = 0xF2U;

	const std::uint8_t STATUS_ADDR    = 0xF3U;
	const std::uint8_t RESET_ADDR     = 0xE0U;

	const std::uint8_t PRESS_ADDR = 0xF7U;
	const std::uint8_t TEMP_ADDR  = 0xFAU;
	const std::uint8_t HUM_ADDR   = 0xFDU;

	const std::uint8_t DIG_LENGTH = 32;

	const std::uint8_t TEMP_DIG_LENGTH = 6;
	const std::uint8_t PRES_DIG_LENGTH = 18;
	const std::uint8_t HUM1_DIG_LENGTH = 1;
	const std::uint8_t HUM2_DIG_LENGTH = 7;


	const std::uint8_t DIG_T1_ADDR = 0x88U;
	const std::uint8_t DIG_T2_ADDR = 0x8AU;
	const std::uint8_t DIG_T3_ADDR = 0x8CU;

	const std::uint8_t DIG_P1_ADDR = 0x8EU;
	const std::uint8_t DIG_P2_ADDR = 0x90U;
	const std::uint8_t DIG_P3_ADDR = 0x92U;
	const std::uint8_t DIG_P4_ADDR = 0x94U;
	const std::uint8_t DIG_P5_ADDR = 0x96U;
	const std::uint8_t DIG_P6_ADDR = 0x98U;
	const std::uint8_t DIG_P7_ADDR = 0x9AU;
	const std::uint8_t DIG_P8_ADDR = 0x9CU;
	const std::uint8_t DIG_P9_ADDR = 0x9EU;

	const std::uint8_t DIG_H1_ADDR = 0xA1U;
	const std::uint8_t DIG_H2_ADDR = 0xE1U;
	const std::uint8_t DIG_H3_ADDR = 0xE3U;
	const std::uint8_t DIG_H4_ADDR = 0xE4U;
	const std::uint8_t DIG_H5_ADDR = 0xE5U;
	const std::uint8_t DIG_H6_ADDR = 0xE7U;

	const std::uint8_t SENSOR_LENGTH = 8;

	/*
	 * TRIMS
	 */

	std::uint16_t dig_T1;
	std::int16_t  dig_T2;
	std::int16_t  dig_T3;

	std::uint8_t dig_H1;
	std::int16_t dig_H2;
	std::uint8_t dig_H3;
	std::int16_t dig_H4;
	std::int16_t dig_H5;
	std::int8_t  dig_H6;

	std::uint16_t  dig_P1;
	std::int16_t   dig_P2;
	std::int16_t   dig_P3;
	std::int16_t   dig_P4;
	std::int16_t   dig_P5;
	std::int16_t   dig_P6;
	std::int16_t   dig_P7;
	std::int16_t   dig_P8;
	std::int16_t   dig_P9;

	std::int32_t t_fine;

	Settings m_settings;


	/*
	 * You can put your own I2C implementation, for now, i'm using HAL for I2C communication
	 */

	float CalculateTemp(std::int32_t raw);
	float CalculateHum(std::int32_t raw);
	float CalculatePress(std::int32_t raw);

	bool ReadingCalibration(void);
	bool ReadSensorData(std::int8_t * data);
	bool Write_reg(std::uint8_t data);
	bool Write_reg(std::uint8_t * data, std::uint8_t size);
	bool Read_reg(std::uint8_t * data, std::uint8_t mem_addr ,std::uint8_t size);

	bool GetDig_T();

};


#endif /* BME280_H_ */
