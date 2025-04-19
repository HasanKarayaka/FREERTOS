
#include "main.h"



extern uint32_t SystemCoreClock;

uint32_t systemClock;

void RCC_Config(void)
{
	RCC->AHB1ENR |= 1 <<3; // AHBq hattını aktif ettik gpıo pini için

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

void GPIO_Config(void)
{
	GPIOD->MODER |= 1 << 24; // 24. biti 1  GPIOD 12 pin output oldu
	GPIOD->MODER &= ~(1 << 25); // 25. biti 0 yapıyoruz
	GPIOD->MODER |= 1 << 26;  // 13. pin output
	GPIOD->MODER &= ~(1 << 27);
	GPIOD->MODER |= 1 << 28; // 14. pin output
	GPIOD->MODER &= ~(1 << 29);
	GPIOD->MODER |= 1 << 30; // 15. pin output
	GPIOD->MODER &= ~(1 << 31);

	GPIOD->OSPEEDR |= 0xFF000000;

}

int main(void)
{
	RCC_Config(); // yukarda yazdığım fonksiyonun rcc paramtrelerini aldı ve altaki yükleme ve atama komutları ile atadı
	SystemCoreClockUpdate();

	GPIO_Config();


  while (1)
  {
	  GPIOD->ODR |= 1 << 12; // 12. pin set edildi
	  GPIOD->ODR |= 1 << 13;
	  GPIOD->ODR |= 1 << 14;
	  GPIOD->ODR |= 1 << 15;

	  for(int i =0; i< 1680000; i++);

	  GPIOD->ODR &= 1<<12; // 12. pin reset oldu
	  GPIOD->ODR &= 1<<13;
	  GPIOD->ODR &= 1<<14;
	  GPIOD->ODR &= 1<<15;


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
