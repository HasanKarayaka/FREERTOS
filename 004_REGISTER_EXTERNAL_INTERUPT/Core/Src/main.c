
#include "main.h"



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
	RCC->AHB1ENR = 0x00000009; //GPIOA ve GPIOD aktif

	GPIOD->MODER = 0x55000000; // GPIOD pin12 den pin15 e akdar
	GPIOD->OTYPER = 0x00000000;
	GPIOD->OSPEEDR = 0xFF000000; // hızını ayarladım
	GPIOD->PUPDR = 0x00000000;




}

void EXTI_Config()
{
	RCC->AHB2ENR = 0x00004000; // syscfg atif :

	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);

	SYSCFG->EXTICR[0] = 0x00000000; // sistem config aktivasyonu,

	__NVIC_SetPriority(EXTI0_IRQn,0); // en öncelikli bu
	__NVIC_SetPriority(EXTI0_IRQn,1);
	__NVIC_SetPriority(EXTI0_IRQn,2);

	EXTI->IMR = 0x00000007; // 3 tane biti int yaptık PA0 PA1 PA2
	EXTI->RTSR = 0x00000007; // yükselen kenar tetik seçtik
}

void EXTI_IRQHandler()
{
	if(EXTI->PR & 0x0000001) // kardeşim gerçekten interup oldumu bunu anlamak için bunu yapıyoruz burda bayrak kaldırıyoruz & bu mantık komutu ile
	{
		GPIOD->ODR = 0x00001000; // şuan ilk ledi yaktım diğerleri söndü
		delay(10000);

		EXTI->PR = 0x00000001; // şimdi bayrağı indirdim

	}
}

void EXTI1_IRQHandler()
{
	if(EXTI->PR & 0x00000002)
	{
		GPIOD->ODR = 0x00002000;
		delay(100000);

		EXTI->PR = 0x00000002;
	}
}

void EXTI2_IRQHandler()
{
	if(EXTI->PR & 0x00000004)
	{
		GPIOD->ODR = 0x0004000;
		delay(100000);

		EXTI->PR = 0x00000004;
	}
}

int main(void)
{
	RCC_Config(); // yukarda yazdığım fonksiyonun rcc paramtrelerini aldı ve altaki yükleme ve atama komutları ile atadı
	SystemCoreClockUpdate();

	GPIO_Config();
	EXTI_Config();


  while (1)
  {
	  GPIOD->ODR = 0x0000F000; // 0 dan 15 e kadar devam eden bitlerde 12 13 14 15. pinleri aktive ettim
	  GPIOD->OTYPER = 0x0000F000; // output atadım
	  GPIOD->OSPEEDR = 0x0000F000; // veri high yaptım pini


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
