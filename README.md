# SmartGarden

Smart garden is an embbeded system made with an educational intention. <br/>
<br/>
The main brain is the STM32F303RB microcontroller, this is a 32 bit cortex M4 microcontroller and is connected via serial UART communication with a raspberry pi that will store the data and will host the web app in order to visualize data<br/>

The hardware architecture is shown in the following figure:<br/>
![Hardware architecture](./images/HardwareArchitectureGardenSensor.jpg) 

## STM32F302RE

STM32F303x8 is a 32 bit Cortex-M4 microcontroller. This project uses the following peripherals:
- ADC
- I2C
- UART
- DMA


### Firmware
The firmware is made using the CMSIS standard except the I2C driver, it uses STM HAL library. The flow chart can be seen in the following picture: <br/>
![Flow chart](./images/gardemeterFlowChart.jpg) 

### Hardware

The PCB is made using KiCad. You can see the project files here: [Analog Reader](https://github.com/juamarCas/AnalogReader)

## Raspberry pi
The RPI part of the project can be seen here: https://github.com/juamarCas/MQTT_middleware

