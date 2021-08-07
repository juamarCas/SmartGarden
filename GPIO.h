#ifndef _GPIO_H
#define _GPIO_H

#include <cstdint>
#include "stm32f3xx.h"


#define	INPUT  		0x0U
#define	OUTPUT  	0x1U
#define	ALTERNATE 0x2U 
#define	ANALOG    0x3U 

#define HIGH 0x1U
#define LOW  0x0U


namespace periph{
	namespace GPIO{
		void set_pin(GPIO_TypeDef * gpio, std::uint32_t pin ,std::uint8_t mode); 
		void set_pin_value(GPIO_TypeDef * gpio, std::uint32_t pin, std::int32_t state);
		void set_afrl(GPIO_TypeDef * gpio, std::uint32_t val, std::uint32_t reg);
		void set_afrh(GPIO_TypeDef * gpio, std::uint32_t val, std::uint32_t reg);
	}	
}

#endif
