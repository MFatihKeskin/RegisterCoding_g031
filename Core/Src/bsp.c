/*
 * 		File: 003_Register Coding - BSP.C
 * 		Date: 19.01.22
 * 		Author: Muhammet Fatih KESKIN
 *
 * */

#include "bsp.h"
#include "stm32g0xx.h"
#include "stdlib.h"
#include "stdio.h"

int SystemCoreClockWithPLL_009(void){//pll ile kartı 64mhz de çalıştırma
	extern uint32_t SystemCoreClock;//extern başka kütüphanelerde var olan bir şeyi dahil etmek için kullanılır. Değer atanamaz
	uint32_t temp=0;
	temp=SystemCoreClock;

	return temp;//pll ile 64Mhz ayarlanmıstı

	/* main:
	  int temp1=SystemCoreClockWithPLL_009();
	*/
}

int SystemCoreClockWithHSI_009(void){//hsi ile kartı 16mhz de çalıştırma
	extern uint32_t SystemCoreClock;//extern başka kütüphanelerde var olan bir şeyi dahil etmek için kullanılır. Değer atanamaz
	uint32_t temp=0;

	HAL_RCC_DeInit(); //pll bağlantısını kaldırır hsi devreye girer.
	SystemCoreClockUpdate();//değiştirilen saat ayarlarını günceller

	temp=SystemCoreClock;

	return temp;//pll devre dışı kaldı. hsi 16 Mhz ile çalışır

	/* main:
	   int temp2=SystemCoreClockWithHSI_009();
	*/
}

int RCC_ConfigWithPLL_012(void){//SYSTEM CLOCK = ((HSE / PLL_m)*PLL_n) / PLL_p = 64MHz (max)

	RCC->PLLCFGR &= 0x00001000; //reset pll config reg
	//[6:4] 000 for pll_m = 1
	RCC->PLLCFGR &= (0U<<4);
	RCC->PLLCFGR &= (0U<<5);
	RCC->PLLCFGR &= (0U<<6);
	//[14:8] 0001000 for pll_n = 8
	RCC->PLLCFGR &= (0U<<8);
	RCC->PLLCFGR &= (0U<<9);
	RCC->PLLCFGR &= (0U<<10);
	RCC->PLLCFGR &= (0U<<12);
	RCC->PLLCFGR &= (0U<<13);
	RCC->PLLCFGR &= (0U<<14);
	//[21:17] 00001 for pll_p=2
	RCC->PLLCFGR &= (0U<<18);
	RCC->PLLCFGR &= (0U<<19);
	RCC->PLLCFGR &= (0U<<20);
	RCC->PLLCFGR &= (0U<<21);

	RCC->PLLCFGR |= (1U<<11);
	RCC->PLLCFGR |= (1U<<17);

	RCC->CR &= 0x00000500; //reset control reg
	RCC->CR |= (1U<<24);//pllon register active
	//bit 25 pll ready flag is active for 0
	while((RCC->CR & (1U<<25)));//wait rcc_cr pllrdy flag return 0

	RCC->CFGR &= 0x00000000; //clock config reg
	//[2:0] cfgr system clock switch is pll
	RCC->CFGR |= (1U<<1);
	while(!(RCC->CFGR & (1U<<1)));

	extern uint32_t SystemCoreClock;//extern başka kütüphanelerde var olan bir şeyi dahil etmek için kullanılır. Değer atanamaz
	uint32_t temp=0;
	temp=SystemCoreClock;

	return temp;

	/*
	mainn:
	extern uint32_t SystemCoreClock;
	uint32_t temp=0;
	temp=SystemCoreClock;

	HAL_RCC_DeInit(); //pll bağlantısını kaldırır hsi devreye girer.
	SystemCoreClockUpdate();//değiştirilen saat ayarlarını günceller

	temp=SystemCoreClock;

	RCC_ConfigWithPLL_012();

	SystemCoreClockUpdate();//değiştirilen saat ayarlarını günceller

	temp=SystemCoreClock;*/
}

void delay(uint32_t time) //if the time is 1600000 then 1 second delay
{
	while(time--);
}

void RCC_ClockEnabled(char port_name){
	if(port_name=='A'){
		//port A clock enable
		RCC->IOPENR &= 0x00000000;//reset value
		RCC->IOPENR |= (1U << 0);
	}
	else if(port_name=='B'){
		//port B clock enable
		RCC->IOPENR |= (1U << 1);
	}
	else{//port C clock enable
		RCC->IOPENR &= 0x00000000;//reset value
		RCC->IOPENR |= (1U << 2);
	}
}

void GPIOx_OutputMode(char port_name, int pin_number){
	if(port_name=='A'){
		RCC_ClockEnabled('A');//port A clock enable and reset values are changes for portA

		//gpio A is output mode (01)
		GPIOA->MODER &= ~(3U << 2*pin_number);//clear bits
		GPIOA->MODER |= (1U << 2*pin_number);

		//gpio A is push-pull type (0)
		GPIOA->OTYPER &= (0U << pin_number);

		//gpio A is very high speed chosed (11)
		GPIOA->OSPEEDR |= (3U << 2*pin_number);

		//gpio A is no pull up-no pull down(00)
		GPIOA->PUPDR &= (0U << 2*pin_number);
	}
	else if(port_name=='B'){
		RCC_ClockEnabled('B');//port B clock enable

		//gpio B is output mode (01)
		GPIOB->MODER &= ~(3U << 2*pin_number);//clear bits
		GPIOB->MODER |= (1U << 2*pin_number);

		//gpio B is push-pull type (0)
		GPIOB->OTYPER &= (0U << pin_number);

		//gpio B is very high speed chosed (11)
		GPIOB->OSPEEDR |= (3U << 2*pin_number);

		//gpio B is no pull up-no pull down(00)
		GPIOB->PUPDR &= (0U << 2*pin_number);
	}
	else{
		RCC_ClockEnabled('C');//port C clock enable

		//gpio c is output mode (01)
		GPIOC->MODER &= ~(3U << 2*pin_number);//clear bits
		GPIOC->MODER |= (1U << 2*pin_number);

		//gpio c is push-pull type (0)
		GPIOC->OTYPER &= 0x00000000;//reset value
		GPIOC->OTYPER &= (0U << pin_number);

		//gpio c is very high speed chosed (11)
		GPIOC->OSPEEDR &= 0x00000000;//reset value
		GPIOC->OSPEEDR |= (3U << 2*pin_number);

		//gpio c is no pull up-no pull down(00)
		GPIOC->PUPDR &= 0x00000000;
		GPIOC->PUPDR &= (0U << 2*pin_number);
	}
}

void GPIOx_InputMode(char port_name, int pin_number){
	if(port_name=='A'){
		RCC_ClockEnabled('A');//port A clock enable and reset values are changes for portA

		//gpioa is input mode (00)
		GPIOA->MODER &= ~(3U << 2 * pin_number);
	}
	else{
		RCC_ClockEnabled('B');//port B clock enable

		//gpiob is input mode (00)
		GPIOB->MODER &= ~(3U << 2 * pin_number);
	}
}

void GPIOx_AnalogMode(char port_name, int pin_number){
	if(port_name=='A'){
		RCC_ClockEnabled('A');//port A clock enable and reset values are changes for portA

		//gpioa is input mode (00)
		GPIOA->MODER |= (3U << 2 * pin_number);
		//gpio A is very high speed chosed (11)
		GPIOA->OSPEEDR |= (3U << 2*pin_number);
	}
	else{
		RCC_ClockEnabled('B');//port B clock enable

		//gpiob is input mode (00)
		GPIOB->MODER |= (3U << 2 * pin_number);
		//gpio A is very high speed chosed (11)
		GPIOB->OSPEEDR |= (3U << 2*pin_number);
	}
}

void GPIOx_AlterneFuncMode(char port_name, int pin_number){
	if(port_name=='A'){
		RCC_ClockEnabled('A');//port A clock enable and reset values are changes for portA

		//gpioa is input mode (00)
		GPIOA->MODER |= (1U << 2 * pin_number);
	}
	else{
		RCC_ClockEnabled('B');//port B clock enable

		//gpiob is input mode (00)
		GPIOB->MODER |= (2U << 2 * pin_number);
	}
}

void GPIO_LedHigh(char port_name, int pin_number){//reset values are changes for portA

	GPIOx_OutputMode(port_name, pin_number);

	if(port_name=='A'){
		for(int s=640000; s>0; s--){
			GPIOC->ODR |= (1U << pin_number);
		}
	}
	else if(port_name=='B'){
		for(int s=640000; s>0; s--){
			GPIOB->ODR |= (1U << pin_number);
		}
	}
	else{//port name is C
		for(int s=640000; s>0; s--){
			GPIOC->ODR |= (1U << pin_number);
		}
	}
}

void GPIO_LedHighSwitch(char port_name, int pin_number){//reset values are changes for portA

	GPIOx_OutputMode(port_name, pin_number);

	switch(port_name){
		case 'A':
			for(int s=640000; s>0; s--){
				GPIOC->ODR |= (1U << pin_number);
			}
			break;
		case 'B':
			for(int s=640000; s>0; s--){
				GPIOC->ODR |= (1U << pin_number);
			}
			break;
		default:
			for(int s=640000; s>0; s--){
				GPIOC->ODR |= (1U << pin_number);
			}
	}
}

void GPIO_LedLow(char port_name, int pin_number){//reset values are changes for portA

	GPIOx_OutputMode(port_name, pin_number);

	if(port_name=='A'){
		for(int s=640000; s>0; s--){
			GPIOA->ODR &= (0U << pin_number);
		}
	}
	else if(port_name=='B'){
		for(int s=640000; s>0; s--){
			GPIOB->ODR &= (0U << pin_number);
		}
	}
	else{//port name is C
		for(int s=640000; s>0; s--){
			GPIOC->ODR &= (0U << pin_number);
		}

	}
}

void GPIO_LedBlink_0015(char port_name, int pin_number){//reset values are changes for portA

	GPIOx_OutputMode(port_name, pin_number);

	if(port_name=='A'){
	    for(;;){
	        for(int s=640000; s>0; s--);//one second loop
	        GPIOA->ODR ^= (1U << pin_number); //xor is changes internal led state
	    }
	}
	else if(port_name=='B'){
	    for(;;){
	        for(int s=640000; s>0; s--);//one second loop
	        GPIOB->ODR ^= (1U << pin_number); //xor is changes internal led state
	    }
	}
	else{//port name is C
		for(;;){
			for(int s=640000; s>0; s--);//one second loop
			GPIOC->ODR ^= (1U << pin_number); //xor is changes internal led state
		}
	}
	/*main:
	 *GPIO_LedBlink_0015('C', 6); //for pc6 internal led blink*/
}

void GPIOx_ExternalInterrupt(char port_name, int pin_number, int priority_level){

	GPIOx_InputMode(port_name, pin_number);

	//external interrupt selection register (choose with mux, where is the external interrupt will come from)
	//EXTICR0 -> 0..3, EXTICR1 -> 4..7 , EXTICR2 -> 8..11..... portA will trigger 0U, portB will trigger 1U
	if(port_name=='A'){
		if(pin_number>=0 && pin_number<=3){//pa0,pa1,pa2,pa3
			EXTI->EXTICR[0] |= (0U << 8*pin_number);
		}
		else if(pin_number>=4 && pin_number<=7){//pa4,pa5,pa6,pa7
			EXTI->EXTICR[1] |= (0U << 8*(pin_number-4));
		}
		else{//pin_number>=8, pa8,pa9,pa10,pa11...
			EXTI->EXTICR[2] |= (0U << 8*(pin_number-8));
		}
	}
	else if(port_name=='B'){
		if(pin_number>=0 && pin_number<=3){//pb0,pb1,pb2,pb3
			EXTI->EXTICR[0] |= (1U << 8*pin_number);
		}
		else if(pin_number>=4 && pin_number<=7){//pb4,pb5,pb6,pb7
			EXTI->EXTICR[1] |= (1U << 8*(pin_number-4));
		}
		else{//pin_number>=8, pb8,pb9,pb10,pb11...
			EXTI->EXTICR[2] |= (1U << 8*(pin_number-8));
		}
	}
	else{
		printf("This port can not use in stm32g0k8tx board");
	}

	//rising trigger selection register(choosed rising edge)
	EXTI->RTSR1 |= (1U << pin_number);
	//interrupt mask register(used to enable a software-filtering of noise on external interrupts)
	EXTI->IMR1 |= (1U << pin_number);
	//this interrupt is absolutely true
	EXTI->RPR1 = (1U << pin_number);

	if(pin_number>=0 && pin_number<=1){
		//enable NVIC(nested vector interrupt control)
		NVIC_SetPriority(EXTI0_1_IRQn,priority_level);
		NVIC_EnableIRQ(EXTI0_1_IRQn);
		//now will going to EXTI0_1_IRQHandler
	}
	else if(pin_number>=2 && pin_number<=3){
		//enable NVIC(nested vector interrupt control)
		NVIC_SetPriority(EXTI2_3_IRQn,priority_level);
		NVIC_EnableIRQ(EXTI2_3_IRQn);
		//now will going to EXTI2_3_IRQHandler
	}
	else if(pin_number>=4 && pin_number<=15){
		//enable NVIC(nested vector interrupt control)
		NVIC_SetPriority(EXTI4_15_IRQn,priority_level);
		NVIC_EnableIRQ(EXTI4_15_IRQn);
		//now will going to EXTI4_15_IRQHandler
	}
	else{
		printf("stm32g0k8tx board don't has this pin_number");
	}

	/*main:
	 * GPIOx_ExternalInterrupt('B',3,0);
	 exti2_3irqHandler:
	 * GPIOx_OutputMode('C', 6);
	 * GPIOC->ODR |= (1U << 6); //turn on pc6 led
     * delay(1600000);
     * GPIOC->ODR &= (0U << 6); //turn on pc6 led
     * EXTI->RPR1 |= (1U << pin_number); or if block -> if(EXTI->RPR1 & (1U << pin_number))*/
}


void EXTI0_1_IRQHandler(void){
	//write here

}

void EXTI2_3_IRQHandler (void){
	//write here

}

void EXTI4_15_IRQHandler(void){
	//write here

}

//if we check any flag in while, ADC is not work
void ADC_Calibration(void){
	ADC1->CR |= (1U << 28); //ADVREGEN = 1 for calibration
	ADC1->CR |= (1U << 31);//ADCAL = 1
	//while((ADC1->ISR & (1U << 11)));//wait until ADCAL is 0    or   ADC1->ISR & (1U << 11) => eocal is 1
}

void ADC_Configuration(void){
	//RES=>00: 12 bits, 01: 10 bits, 10: 8 bits, 11: 6 bits
	ADC1->CFGR1 |= (0U << 3); // RES=12 bits
	ADC1->CFGR1 |= (1U << 13); // cont =0 for single conversion mode
	ADC1->ISR &= ~(1U << 0); //clear the adrdy bit
}

uint16_t ADC_EnabledWithPriority(char port_name, int pin_number,int priority){
	uint16_t adc_value = 0; //its must be bigger than resolution bits

	GPIOx_AlterneFuncMode(port_name, pin_number);
	RCC->APBENR2 |= (1U << 20); //ADC only starting APB clock line

	ADC_Calibration();
	ADC_Configuration();

	ADC1->CR |= (1U << 0); //ADEN = 1
	//while((ADC1->ISR & (1U << 0))); //wait until ADRDY = 1
	ADC1->CHSELR |= (1U << pin_number); //analog input channel 7 selection
	ADC1->SMPR |= (2U << 4); //001: 3.5 ADC clock cycles

	NVIC_SetPriority(ADC1_IRQn, priority);
	NVIC_EnableIRQ(ADC1_IRQn);

	ADC1->CR |= (1U << 2); //ADSTART = 1

	//while (!(ADC1->ISR & (1U << 2)));//wait until EOC flag is set
	adc_value = (uint16_t)ADC1->DR;

	return adc_value;

	/*
	  main-> while 1:
	  uint16_t read_adc = ADC_EnabledWithPriority('A', 7, 0);
	  if(read_adc>2000){
		  GPIO_LedHigh('C', 6);
	  }
	  else{
		  GPIO_LedLow('C', 6);
	  }
	*/
}

void initTIM2(int priority){
	RCC->APBENR1 |= (1U << 0); //TIM2 enable on APB bus

	TIM2->CR1 = 0;			//control register is 0 for just in case
	TIM2->CR1 |= (1U << 7); //arpe register is buffered
	TIM2->CNT = 0;			// counter register is 0

	TIM2->PSC = 999; //prescaler
	TIM2->ARR = 16000; // //auto reload register

	TIM2->DIER |= (1U << 0); // update interrupt enable

	TIM2->CR1 |= (1U << 0); // TIM2 enable

	NVIC_SetPriority(TIM2_IRQn, 0); // TIM2 NVIC enable
	NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {

}

void initTIM3(int priority){
	RCC->APBENR1 |= (1U << 1); //TIM3 enable on APB bus

	TIM3->CR1 = 0;			//control register is 0 for just in case
	TIM3->CR1 |= (1U << 7); //arpe register is buffered
	TIM3->CNT = 0;			// counter register is 0

	TIM3->PSC = 999; //prescaler
	TIM3->ARR = 16000; // //auto reload register

	TIM3->DIER |= (1U << 0); // update interrupt enable

	TIM3->CR1 |= (1U << 0); // TIM3 enable

	NVIC_SetPriority(TIM3_IRQn, 0); // TIM3 NVIC enable
	NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void) {

}










void _print(int fd, char *ptr, int len){
	(void)fd;
	for(int i=0; i<len ; ++i){
		printChar(ptr[i]);
	}
}

void printChar(uint8_t c){
    USART2->TDR =(uint16_t)c;
    while(!( USART2->ISR & (1<<6)));
}

void BSP_UART_init(uint32_t baud){
	RCC->IOPENR  |= (1U << 0);
	RCC->APBENR1 |= (1U << 17);

	//SETUP PA2 AS AF1
	GPIOA->MODER &= ~(3U<< 2*2);
	GPIOA->MODER |=  (2U<< 2*2);
	//CHOOSE AF1 FROM MUX
    GPIOA->AFR[0] &= ~(0xFU << 4*2);
    GPIOA->AFR[0] |= (1U<< 4*2);

	//SETUP PA3 AS AF1
	GPIOA->MODER &= ~(3U<< 2*3);
	GPIOA->MODER |=  (2U<< 2*3);
	//CHOOSE AF1 FROM MUX
    GPIOA->AFR[0] &= ~(0xFU << 4*3);
    GPIOA->AFR[0] |= (1U<< 4*3);

    //SETUP UART2
    USART2->CR1  = 0; //RESET UART2 WITH CR1
    USART2->CR1 |= (1 << 3); //TE
    USART2->CR1 |= (1 << 2); //RE
    USART2->CR1 |= (1 << 5); //RXNEIE
    USART2->BRR  = (uint16_t)(SystemCoreClock / baud);
    USART2->CR1 |= (1 << 0); //UE
/*
    NVIC_SetPriority(USART2_IRQn,0);
    NVIC_EnableIRQ(USART2_IRQn);
*/
}

void USART2_IRQHandler(uint16_t data){
	 data = (uint8_t)USART2 ->RDR;
	printChar(data);
}


