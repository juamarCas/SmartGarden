#include "GPIO.h"

namespace periph{
	namespace GPIO{
		void set_pin(GPIO_TypeDef * gpio, std::uint32_t pin ,std::uint8_t m){
			gpio->MODER &= ~((1U << pin * 2) | (1U << (pin * 2 + 1))); 
			if(m == INPUT) return; 		
			gpio->MODER |= (m << (pin * 2)); 								
		 }
			
			void set_pin_value(GPIO_TypeDef * gpio, std::uint32_t pin, std::int32_t state){	
				if(state == LOW) gpio->ODR &= ~(1U << pin);
				else gpio->ODR |= (1U << pin); 		
			}
	
			void set_afrl(GPIO_TypeDef * gpio, std::uint32_t val, std::uint32_t reg){
				gpio->AFR[0] |= (val << reg);  
			}
		
			void set_afrh(GPIO_TypeDef * gpio, std::uint32_t val, std::uint32_t reg){
				gpio->AFR[1] |= (val << reg);  
			}
    }
		
	
	}
	
