#ifndef SRC_BSP_H_
#define SRC_BSP_H_

#include "stm32g0xx.h"
//write here
/**************************************** CLOCK CONFİGURATİON ***********************************/
int SystemCoreClockWithPLL_009(void);
int SystemCoreClockWithHSI_009(void);

/**************************************** MANUAL DELAY *****************************************/
void delay(uint32_t time);

/**************************************** RCC CONFİGURATİON ***********************************/
int RCC_ConfigWithPLL_012(void);
void RCC_ClockEnabled(char port_name);

/**************************************** GPIO MODE ******************************************/
void GPIOx_OutputMode(char port_name, int pin_number);
void GPIOx_InputMode(char port_name, int pin_number);
void GPIOx_AnalogMode(char port_name, int pin_number);
void GPIOx_AlterneFuncMode(char port_name, int pin_number);

/**************************************** GPIO LED EXERCİSE ***********************************/
void GPIO_LedBlink_0015(char port_name, int pin_number);
void GPIO_LedLow(char port_name, int pin_number);
void GPIO_LedHigh(char port_name, int pin_number);
void GPIO_LedHighSwitch(char port_name, int pin_number);


/**************************************** EXTERNAL INTERRUPT ***********************************/
void GPIOx_ExternalInterrupt(char port_name, int pin_number, int priority_level);
void EXTI0_1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void EXTI4_15_IRQHandler(void);

/**************************************** ADC CONFİGURATİON ***********************************/
//if we check any flag in while, ADC is not work
void ADC_Calibration(void);
void ADC_Configuration(void);
uint16_t ADC_EnabledWithPriority(char port_name, int pin_number,int priority);//from lab-7

/**************************************** TIMER CONFİGURATİON ***********************************/
void initTIM2(int priority);
void TIM2_IRQHandler(void);
void initTIM3(int priority);
void TIM3_IRQHandler(void);
void initTIM4(int priority);
void TIM4_IRQHandler(void);

/**************************************** UART CONSOLE PRİNT ***********************************/
void BSP_UART_init(uint32_t baud);
void _print(int fd, char *ptr, int len);
void printChar(uint8_t c);
void USART2_IRQHandler(uint16_t data);



#endif






















/*

*/

