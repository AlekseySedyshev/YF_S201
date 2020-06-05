//MCU STM32F042
//I2C LCD Pinout  - PB6 - SCL, PB7 - SDA

#include "stm32f0xx.h"        // Device header
#include "SSD1306.h"

#define I2C_400				1		//		400 kHz i2C Freq
#define I2C_GPIOB			1	//		i2C PB6,PB7


#define YF_S201 373		//Here should be put constant for your counter, it.s need for calibrate

uint8_t i,k,test=0;
uint8_t IRQ_Flag,sec_tic;
uint16_t TimingDelay,led_count,ms1000;
uint32_t old_value,delta;
float l_min;
volatile uint32_t global_value;
uint8_t gl_val_flag=0;

void TimingDelayDec(void) 																													{
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;}
 if (!ms1000) {ms1000=1000;sec_tic=1;}
 led_count--;ms1000--;
 }

void TIM17_IRQHandler(void)																													{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void delay_ms (uint16_t DelTime) 																										{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}
void TIM2_IRQHandler(void)																													{
		if (TIM2->SR & TIM_SR_UIF) {
			global_value++;
			gl_val_flag=1;
  		TIM2->SR &=(~TIM_SR_UIF);
		}
}	
void initial (void)																																{
//---------------TIM17------------------
  RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 8 MHz - 1 msek
  TIM17->PSC = 8000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 																					//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;																					//Pb0-Out 
//------------I2C1 GPIOB_SETTING ---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOBEN;
	GPIOB->MODER 		|=GPIO_MODER_MODER6_1 		| GPIO_MODER_MODER7_1; 							// Alt -mode /Pb6 -SCL , Pb7- SDA
	GPIOB->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR6 	| GPIO_OSPEEDER_OSPEEDR7;
	GPIOB->OTYPER		|=GPIO_OTYPER_OT_6 				| GPIO_OTYPER_OT_7;
	GPIOB->AFR[0] 	|=(1<<GPIO_AFRL_AFRL6_Pos) |(1<<GPIO_AFRL_AFRL7_Pos);  				// I2C - Alternative PB7, PB6

	RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
//============I2C_400khz===============
	I2C1->TIMINGR |=(0x0	<<I2C_TIMINGR_PRESC_Pos); 	//400 kHz - I2C bus speed
	I2C1->TIMINGR |=(0x9	<<I2C_TIMINGR_SCLL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLH_Pos);
	I2C1->TIMINGR |=(0x1	<<I2C_TIMINGR_SDADEL_Pos);
	I2C1->TIMINGR |=(0x3	<<I2C_TIMINGR_SCLDEL_Pos);

	I2C1->CR2 &=(~I2C_CR2_HEAD10R) & (~I2C_CR2_ADD10);
	I2C1->CR1 |=I2C_CR1_PE;
//===============TIM2_CH1 for Water Flow Sensor YF-S201===================
	RCC->AHBENR  		|= RCC_AHBENR_GPIOAEN; 							//
	GPIOA->OSPEEDR 	|= GPIO_OSPEEDR_OSPEEDR0;						// 50 MHz - speed
	GPIOA->PUPDR 		|= GPIO_PUPDR_PUPDR0_0;							//Pull Up
	GPIOA->MODER 		|= GPIO_MODER_MODER0_1; 						// alternative
	GPIOA->AFR[0] 	|= 2<< GPIO_AFRL_AFRL0_Pos;  				//Pa0 - Alternative 2 - TIM2_Ch1

	RCC->APB1ENR |=RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 0;  																		// 8MHZ - in
	TIM2->ARR = (YF_S201*1000)-1; 	 //1000 liters
	TIM2->CCR2 = 0;  
		
	TIM2->SMCR 		|= 0b101<<TIM_SMCR_TS_Pos;						//Ti1_FP1 - Connected
	TIM2->SMCR 		|= 0b111<<TIM_SMCR_SMS_Pos;						//External Clock - SMS mode 0b111
	TIM2->CCMR1 	|=  TIM_CCMR1_CC1S_0;									//
	TIM2->CCER 		&=~(TIM_CCER_CC1P );  								//Fall edge
	TIM2->CCER 		&=~(TIM_CCER_CC1NP); 									//Not invert
	TIM2->CNT			=0;																		//Drop down counter
	TIM2->DIER 		|= TIM_DIER_UIE; 											//Interrupt Enable
	TIM2->CR1 		|= TIM_CR1_CEN;	 											//Timer2 On
	NVIC_EnableIRQ (TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn,0x01);

//=======================================================================

__enable_irq ();	
} 

int main(void)
{
initial();
delay_ms (100);	
LCD_Init();LCD_Clear();
LCD_Gotoxy (20,0);LCD_PrintStr(" TEST RC522 ",1);

//-----------------------------initial data----------------------------------

while (1)  /* Main loop */
{
	
	if (sec_tic) {	sec_tic=0;
		
		if(gl_val_flag==0) 	{		//Normal
				delta= (TIM2->CNT - old_value);
			}
		else 								{		//1000 liter update
				delta= (YF_S201*1000 - old_value + TIM2->CNT);
				gl_val_flag=0;
		}
		
	old_value =	TIM2->CNT;
	l_min=(delta*60)/YF_S201;
	LCD_Gotoxy (1,1); LCD_PrintStr("Pulse Counter ",0);LCD_PrintDec(TIM2->CNT,0);LCD_PrintStr("     ",0);
	LCD_Gotoxy (1,2); LCD_PrintStr("Value ",0);LCD_PrintDec((global_value*1000 + old_value)/YF_S201,0);LCD_PrintStr(",",0);LCD_PrintDec((float)(((global_value*1000 + old_value)%YF_S201)*2.2),0);LCD_PrintStr(" l.     ",0);	
	LCD_Gotoxy (1,3); LCD_PrintStr("Cons. ",0);LCD_PrintDec((delta*60)/YF_S201,0);LCD_PrintStr(",",0);LCD_PrintDec((float)(((delta*60)%YF_S201)*2.2),0);LCD_PrintStr(" l/min.     ",0);	
	
}


} // end - main loop 
} // end - Main  
	
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
