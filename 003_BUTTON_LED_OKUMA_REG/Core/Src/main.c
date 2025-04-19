
#include "main.h"

int count = 0;

void delay(uint32_t time)
{
	while(time--);
}
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
	RCC->AHB1ENR = 0x00000009; // GPIOA ve GPIOD aktif

	GPIOD->MODER = 0x55000000; // 12 13 14 15 dijital ouptup
	GPIOD->OTYPER = 0x00000000; // 003_BUTTON_LED_OKUMA_REG push pull
	GPIOD->OSPEEDR = 0xFF000000; // pin 100mhz
	GPIOD->PUPDR = 0x00000000;
}


int main(void)
{
	RCC_Config(); // yukarda yazdığım fonksiyonun rcc paramtrelerini aldı ve altaki yükleme ve atama komutları ile atadı
	SystemCoreClockUpdate();

	GPIO_Config();


  while (1)
  {

	  if(GPIOA->IDR & 0x00000001) // butonda arg varsa engellemek için
	  {
		  while(GPIOA->IDR & 0x00000001);
		  delay(1680000);

		  if(count % 2 == 0)
		  {
			  GPIOD->ODR = 0x00000000; // bura butona 2 kere basılınca resete çekiyor
		  }
		  else
			  GPIOD->ODR = 0x0000F000; // burda sete çekiyor 1 kere bastı set oldu 3. basması yeniden set olması demek


	  }

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
