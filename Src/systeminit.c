/*System Initialization functions
 * Contains code to configure clocks
 *
 * Author : Oki Agaya Okwiri
 */
#include <stdint.h>
#include "system_stm32f4xx.h"
#include "stm32f4xx.h"
#include "logger.h"

LogLevel system_log_level = LOG_LEVEL_DEBUG;
uint32_t SystemCoreClock = 72000000; //72 MHz


//HCLK = 72MHz
//PLL: M=4; N= 72; P=2; Q=3
//AHB Prescaler =1
//APB1 Prescaler =2
//APB2 Prescaler =2
//MCO1 Prescaler =2
static void configure_clock(void)
{

	//CONFIGURE flash latency before setting up the clocks
	MODIFY_REG(FLASH->ACR,
			FLASH_ACR_LATENCY,
			_VAL2FLD(FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS)
			);

	//Enable HSC
	SET_BIT(RCC->CR, RCC_CR_HSEON);

	//Wait till HSE is stable
	while(!READ_BIT(RCC->CR, RCC_CR_HSERDY));
	//Configure PLL
	MODIFY_REG(RCC->PLLCFGR,
			RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLP,
			_VAL2FLD(RCC_PLLCFGR_PLLM, 4) | _VAL2FLD(RCC_PLLCFGR_PLLN, 72) | _VAL2FLD(RCC_PLLCFGR_PLLQ, 3) | RCC_PLLCFGR_PLLSRC_HSE
		);

	//Enable PLL
	SET_BIT(RCC->CR, RCC_CR_PLLON);
	//wait until PLL is stable
	while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));

	//switches system to use PLL as a clock source
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_SW,
			_VAL2FLD(RCC_CFGR_SW,RCC_CFGR_SW_PLL)
			);

	//configure AHB Prescaler

	// Configures PPRE1 = 2, (PPRE2 = 1, HPRE = 1 by default).
		MODIFY_REG(RCC->CFGR,
			RCC_CFGR_PPRE1,
			_VAL2FLD(RCC_CFGR_PPRE1, 4)
			);
	// Waits until PLL is used.
		while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	//Disable HSI to save power
		CLEAR_BIT(RCC->CR, RCC_CR_HSION);

}
void configure_mco1(void)
{
	//configure mco1 to take the clock source from the PLL

	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE,
			_VAL2FLD(RCC_CFGR_MCO1,3) | _VAL2FLD(RCC_CFGR_MCO1PRE,4)
			);
	//enable clock to GPIOA SINCE MCO1 is PA8
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
	//Set speed to medium speed
	MODIFY_REG(GPIOA->OSPEEDR,
			GPIO_OSPEEDR_OSPEED8,
			_VAL2FLD(GPIO_OSPEEDR_OSPEED8, 1)
		);
	//set alternate function mode
	MODIFY_REG(GPIOA->MODER,
			GPIO_MODER_MODER8,
			_VAL2FLD(GPIO_MODER_MODER8, 2)
		);
}
void SystemInit(void)
 {
	//configure_mco1();
	configure_clock();
 }
