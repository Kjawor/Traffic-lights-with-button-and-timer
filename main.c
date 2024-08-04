#include <inttypes.h>
#include <stdbool.h>


struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
  //structure that holds all the RCC registers that are responsible for managing the clock and reset functionalities of peripherals
  //with some of these registers we are able to turn on peripherals that are turned off initially

};



#define RCC ((struct rcc*) 0x40023800)

struct gpio{
	volatile uint32_t MODER,OTYPER,OSPEEDR,PUPDR,IDR,ODR,BSRR,LCKR,AFR[2];
};

#define GPIOD ((struct gpio *) 0x40020C00)
#define GPIOA ((struct gpio *) 0x40020000)

struct nvic{
	volatile uint32_t ISER[8U],RESERVED0[24U],ICER[8U], RESERVED1[24U],  ISPR[8U], RESERVED2[24U],
	ICPR[8U], RESERVED3[24U], IABR[8U], RESERVED4[56U], IP[240U], RESERVED5[644U], STIR;
};

#define NVIC ((struct nvic *) 0xE000E100)

struct tim{
	volatile uint32_t CR1,CR2,SMCR,DIER,SR,EGR,CCMR1,CCMR2,CCER,CNT,PSC,ARR,RESERVED0,CCR1,CCR2,CCR3,CCR4,RESERVED1,DCR,DMAR,TIM2_OR,
	TIM5_OR;
};

#define TIM2 ((struct tim*) 0x40000000)

struct systick{
	volatile uint32_t CTRL, LOAD,VAL,CALIB;
};
#define SYSTICK ((struct systick*) 0xE000E010)

volatile uint8_t status;
volatile int counter = 0;

#define FREQ 16000000 //clock freq
void timstr(void){
	RCC->APB1ENR |=(1U<<0); //turn on timer clock

	NVIC->IP[7] = (1U<<4); //set priority for timer interrupt
	NVIC->ISER[28>>5UL] = (1U<<28); //enable timer interrupt in vector table

	TIM2->CR1 &= ~(1U<<0); //disable timer
	RCC->APB1RSTR |=(1U<<0); //reset timer
	RCC->APB1RSTR &=~(1U<<0); //clear reset bit timer

	TIM2->PSC= 15; //1 us
	TIM2->ARR= 1000; //1ms
	TIM2->EGR |=(1U<<0); //reset timer and update registers
	TIM2->DIER |=(1U<<0); //enable interrupt
	TIM2->CR1 |=(1U<<0); //enable timer




}

void systick_init(uint32_t ticks) {
	RCC->APB2ENR |= (1UL << 14); //enable sysconfig clock
	if(ticks > 0xffffff) return; //check if the ticks value fits 24 bits
	SYSTICK->LOAD = ticks-1; //set the value systick will count down from
	SYSTICK->VAL = 0; //make sure the current value in systick is empty
	SYSTICK->CTRL |= (1U<<0) |(1U<<1) | (1U<<2); //enable: timer, interrupt and internal clock source
}


volatile uint32_t tick; //variable to hold the amount of times the timer interrupt occurred
volatile uint32_t laststate; //stores the state of the previous interrupt to be used for the current one
volatile uint32_t debouncedbuttonstate; //stores the state of the button after successful debouncing

void delay(uint32_t ms) {
	counter = 0; //reset counter
	while(counter < ms); //loop until counter reaches the wanted time in ms
}


void TIM2_IRQHandler(void){
	if(TIM2->SR & (1U<<0)) { //check if interrupt flag was raised after timer finished up/down counting
		TIM2->SR &=~(1U<<0); //clear the interrupt flag

		uint32_t reading = GPIOA->IDR & (1U<<0); //get the current reading at the time of the interrupt

		if(reading !=laststate) {
			tick =0; //if the reading is not the same as the previous interrupt it means a change in the
			//state of the button occurred which means the button switched from HIGH to LOW
			//can mean a bounce
		}
		else{
			tick++; //if the current reading is the same as the reading from the previous interrupt
			//we want to increment the counter until it reaches the desired value of 50 which is 50 ms
			//of the button being in the same state
		}

		if(tick > 50) { //if the state of the button remained the same for 50 ms
			if(reading !=debouncedbuttonstate) { //check if the current reading is the same as the previous reading
				//when the button was successfully debounced last time
				//this will mean that holding down the button wont change the state of the LED
				debouncedbuttonstate = reading;
				//if the current reading is not not the same as the previous successful debounce we want to now update it
				if(debouncedbuttonstate && GPIOD->ODR & (1U<<12)) { //if debounced button is set
					//if the button pin in the IDR register is set it means the button state is HIGH
					//this means the button was pressed/is being pressed down
					//when the button is pressed and the current LED that is on is the green LED we want to:
					//create a small delay then make the traffic light switch to orange to simulate
					//a pedestrian button being pressed in traffic lights
					delay(500);
					status =1;
				}
			}
		}
		laststate = reading;//update the laststate variable at the end of each timer interrupt



	}
}

void SysTick_Handler(void){
	counter++; //at each systick interrupt happening every 1 ms increment a counter.
}


int main(){
RCC->AHB1ENR |=(1U<<0); //enable gpioa
RCC->AHB1ENR |=(1U<<3); //enable gpiod

GPIOD->MODER |=(1U<<26); //output for orange
GPIOD->MODER |=(1U<<24); //output for green
GPIOD->MODER |=(1U<<28); //output for red

systick_init(FREQ/1000);

timstr();

status =0;

while(1){


	//make the lights have a fade in effect somehow

	switch(status){
	case(0): //green light
			GPIOD->ODR |=(1U<<12);
			for(int i =0; i< 5000 && status == 0; i++) {
				delay(1);
			}

			GPIOD->ODR &= ~(1U<<12);

			if(status == 0) status = 1;
			break;
	case(1): //orange
			GPIOD->ODR &= ~(1U<<12);
			GPIOD->ODR |= (1U<<13);
			delay(2000);
			GPIOD->ODR &= ~(1U<<13);
			status = 2;
			break;
	case(2): //red
			GPIOD->ODR |= (1U << 14);
			delay(7000);

			status =3;
			break;
	case(3): //red yellow
			GPIOD->ODR |= (1U << 13);
			delay(1500);
			GPIOD->ODR &= ~(1U << 13);
			GPIOD->ODR &= ~(1U << 14);
			status = 0;
			break;
	}


}

}
