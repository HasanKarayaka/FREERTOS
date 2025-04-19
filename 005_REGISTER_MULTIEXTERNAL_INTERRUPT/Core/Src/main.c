
#include "main.h"



extern uint32_t SystemCoreClock;

uint32_t systemClock;

void delay(uint32_t time)
{
	while(time--);
}
void RCC_Config(void)
{

	RCC-> CR &= ~( 1 << 0); // HSI OFF
	RCC->CR |= 1 <<16; // 16 bit sola öteleyip 1 yazıyor bu usermanuldeki 16. bit HSE biti bu biti 1 e çektik ki hse aktif olsun
	while(!( RCC->CR & (1 << 17)) ); // 17. bitte bayrak var ve diyor ki bu bayrak kalkmadan hse aktif olmuyor o zaman bu bayrağı bekliyorum kalksın "&" ifadesi burda karşılaştırma yapıyor rcc deki 17 bit 1 mi değil mi
	RCC->CR |= 1 <<19; // güvenlik biti
	RCC->PLLCFGR = 0x00000000;
	RCC->PLLCFGR |= (1 << 22); //PLL HSE
	RCC->PLLCFGR |= (4 << 0); //yukarda yapıaln işlemin kısa hali  PLL M 4
	RCC-> PLLCFGR |= (168 << 6); // 6 bit sola öteledik PLLN konumuna geldik burda da 168 yazdık  PLL n 168

	RCC-> PLLCFGR |= ( 1 << 24); // PLL ON
	while(!( RCC->CR & (1 << 25)) ); // Wait PLL active

	RCC-> CFGR |= (1 << 1); // System Cloack is PLL
	while(!(RCC->CFGR & (1 << 1)));


}

void GPIO_Config()
{
	RCC->AHB1ENR = 0x00000009; // kanalı aktif ettim

	GPIOD->MODER =0x55000000; // modu output verdim
	GPIOD->OTYPER = 0x00000000;
	GPIOD->OSPEEDR = 0xFF000000;

}

void EXTI_Config()
{
	RCC->AHB2ENR = 0x00004000;

	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);

	SYSCFG->EXTICR[0] = 0x00000000;
	SYSCFG->EXTICR[1] = 0x00000000;
	SYSCFG->EXTICR[2] = 0x00000000;

	NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_SetPriority(EXTI1_IRQn,1);
	NVIC_SetPriority(EXTI2_IRQn,2);

	EXTI->IMR = 0x00000007;
	EXTI->RTSR = 0x00000007;

}

void EXTI0IRQHandler()
{
	if(EXTI->PR &  (1<<0))
	{
		int i =0;

		do{
			GPIOD->ODR = 0x00001000;
			delay(10000);
			GPIOD->ODR = 0x00000000;
			i++;
		}
		while(i<5);

		EXTI->PR = (1 <<0);
	}
}

void EXTI1IRQHandler()
{
	if(EXTI->PR &  (1<<1))
	{
		int i =0;

		do{
			GPIOD->ODR = 0x00002000;
			delay(10000);
			GPIOD->ODR = 0x00000000;
			i++;
		}
		while(i<5);

		EXTI->PR = (1 <<1);
	}
}

void EXTI2RQHandler()
{
	if(EXTI->PR &  (1<<2))
	{
		int i =0;

		do{
			GPIOD->ODR = 0x00008000;
			delay(10000);
			GPIOD->ODR = 0x00000000;
			i++;
		}
		while(i<5);

		EXTI->PR = (1 <<2);
	}
}


int main(void)
{

	RCC_Config(); // yukarda yazdığım fonksiyonun rcc paramtrelerini aldı ve altaki yükleme ve atama komutları ile atadı
	SystemCoreClockUpdate();

  while (1)
  {

  }

}







void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {

  }

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
