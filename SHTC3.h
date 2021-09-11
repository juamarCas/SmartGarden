/*
 * SHTC3.h
 *
 *  Created on: Sep 4, 2021
 *      Author: yiond
 */

#ifndef SRC_SMARTGARDEN_SHTC3_H_
#define SRC_SMARTGARDEN_SHTC3_H_
#include <cstdint>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"


class SHTC3 {
public:

private:
	I2C_HandleTypeDef * m_i2c;
	std::uint8_t m_addr;


	/*addresses are inverted because of the way i2c sends the data*/
	/*ex -> C8EF is id_addr and i'm sendig EFC8 that is the actual addr*/
	const std::uint16_t id_cmd      = 0xC8EFU;
	const std::uint16_t wake_up_cmd = 0x1735U;

	bool Write_reg(std::uint8_t data);
	bool Write_reg(std::uint8_t * data, std::uint8_t size);
	bool Read_reg(std::uint8_t * data, std::uint8_t mem_addr ,std::uint8_t size);
	bool Read_reg16(std::uint8_t * data, std::uint8_t * mem_addr ,std::uint8_t size);

public:
	/*
	 * read the specific code for the sensor, not entirely, so the return should be: 0x0407
	 */
	std::uint16_t Read_ID(void);
	void begin();
	void sleep(bool sleep);
	void Read_sensor(std::uint8_t &temp, std::uint8_t &hum);
	SHTC3(I2C_HandleTypeDef * i2c, std::uint8_t addr = 0x70);
};

#endif /* SRC_SMARTGARDEN_SHTC3_H_ */
