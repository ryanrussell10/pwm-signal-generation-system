//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//
// Extended for Lab Project By:
// Ryan Russell
// V00873387
// November 20, 2020

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is code for the Final Lab Project.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "assert.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 and TIM3 timers: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
#define myTIM3_PRESCALER ((uint16_t)0xBB7F) // 47999 in decimal (1 KHz)

/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

// Function definitions
void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void myLCD_Init(void);
void updateLCD(unsigned int cur_freq, uint16_t cur_res);
void sendToLCD(uint16_t ODR_data);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void TIM2_IRQHandler(void);
void EXTI0_1_IRQHandler(void);

uint16_t resistance = 0;
unsigned int period_us = 0;
unsigned int freq = 0;
unsigned int voltage = 0;


int main(int argc, char* argv[]) {

	//trace_printf("This is Part 2 of Introductory Lab...\n");
	//trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */
	myGPIOC_Init();		/* Initialize I/O port PC */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myTIM3_Init();		/* Initialize timer TIM3 */
	myEXTI_Init();		/* Initialize EXTI */
	myADC_Init();   	/* Initialize ADC */
	myDAC_Init();   	/* Initialize DAC */
	myLCD_Init();		/* Initialize LCD */

	while (1) {

		// Trigger ADC conversion and test EOC flag
		ADC1->CR |= 0x00000004;
        while ((ADC1->ISR & 0x00000004) == 0);

        // Calculate voltage and then set voltage from converted ADC1 data
		voltage = (unsigned int)(ADC1->DR);
		DAC->DHR12R1 = voltage;

		//trace_printf("Voltage: %d\n", (int)voltage);

		// Calculate resistance
		float res = (5000 * voltage) / 4095;
		resistance = ((uint16_t)res);

		//trace_printf("Frequency: %d\n", (int)freq);
		//trace_printf("Resistance: %d\n\n", (int)resistance);
		updateLCD(freq, resistance);
	}
	return 0;
}


void myGPIOA_Init() {

	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA1 as input (555 Timer) */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	/* Configure PA4 as analog (DAC Analog) */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER |= 0x00000300;

	/* Ensure no pull-up/pull-down for PA4 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
}


void myGPIOB_Init() {

	/* Enable clock for GPIOB peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Configure PB4->PB6 and PB8->PB15 as output */
	// Relevant register: GPIOA->MODER
	GPIOB->MODER = 0x55551500;

	/* Ensure no pull-up/pull-down for PB */
	// Relevant register: GPIOA->PUPDR
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
}


void myGPIOC_Init() {

	/* Enable clock for GPIOC peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	/* Configure PC1 as analog */
	// Relevant register: GPIOC->MODER
	GPIOC->MODER |= 0x0000000C;

	/* Ensure no pull-up/pull-down for PC1 */
	// Relevant register: GPIOC->PUPDR
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}


void myADC_Init() {

	// Enable ADC clock
    RCC->APB2ENR |= 0x00000200;

    // Configure ADC to measure analog voltage signal continuously
    ADC1->CFGR1 &= ~(0x00000020);
    ADC1->CFGR1 |= 0x00002000;
    ADC1->SMPR  |= 0x00000007;

    // Maps ADC to PC1
    ADC1->CHSELR |= 0x00000800;

    // Set 12 bit resolution
    ADC1->CR |= 0x80000000;
    while ((ADC1->CR & 0x80000000) != 0);
    ADC1->CR |= 0x00000001;
    while ((ADC1->ISR & 0x00000001) == 0);
}


void myDAC_Init() {

	// Enable clock for DAC Peripheral and set GPIOC
    RCC->APB1ENR |= 0x20000000;
    GPIOC->MODER |= 0x00000300;

    DAC->CR &= 0xFFFFFCFF;

    // Set DAC Channel to known state, then set enable bit
    DAC->CR |= 0x00000000;
    DAC->CR |= 0x00000001;
}


void myLCD_Init() {

	// Initialization command 1: DL = 1, N = 1, F = 0
	sendToLCD(0x3800);

	// Initialization command 2: D = 1, C = 0, B = 0
	sendToLCD(0x0C00);

	// Initialization command 3: I/D = 1, S = 0
	sendToLCD(0x0600);

	// Initialization command 4: clear display
	sendToLCD(0x0100);

	// Access top row of LCD (frequency row)
	sendToLCD(0x8000);

	// Set first character "F"
	sendToLCD(0x4620);

	// Set second character ":"
	sendToLCD(0x3A20);

	// Access bottom row of LCD (resistance row)
	sendToLCD(0xC000);

	// Set first character "R"
	sendToLCD(0x5220);

	// Set second character ":"
	sendToLCD(0x3A20);
}


void updateLCD(unsigned int cur_freq, uint16_t cur_res) {

	char freqString[4] = {'0', '0', '0', '0'};
	char resString[4]  = {'0', '0', '0', '0'};

	// Convert to ASCII using the offset of 0x30 ('0')
	freqString[0] = cur_freq / 1000 + '0';
	freqString[1] = ((cur_freq % 1000) / 100) + '0';
	freqString[2] = ((cur_freq % 100) / 10) + '0';
	freqString[3] = cur_freq % 10 + '0';

	resString[0] = cur_res / 1000 + '0';
	resString[1] = ((cur_res % 1000) / 100) + '0';
	resString[2] = ((cur_res % 100) / 10) + '0';
	resString[3] = cur_res % 10 + '0';

	// Access top row of LCD (frequency row)
	sendToLCD(0x8200);

	// Set all four digits of the current frequency value
	int i = 0;
	for (i = 0; i < 4; i++) {
		uint16_t ODR_data = ((freqString[i] << 8) + 0x20);
		sendToLCD(ODR_data);
	}

	// Set second last character "H"
	sendToLCD(0x4820);

	// Set last character "z"
	sendToLCD(0x7A20);

	// Access bottom row of LCD (resistance row)
	sendToLCD(0xC200);

	// Set all four digits of the current frequency value
	int j = 0;
	for (j = 0; j < 4; j++) {
		uint16_t ODR_data = ((resString[j] << 8) + 0x20);
		sendToLCD(ODR_data);
	}

	// Set second last character "O"
	sendToLCD(0x4F20);

	// Set last character "h"
	sendToLCD(0x6820);
}


void sendToLCD(uint16_t ODR_data) {

	// Send data, RS bit, and R/W bit
	GPIOB->ODR = ODR_data;

	// Assert Enable
	GPIOB->ODR |= 0x0010;

	// Wait for Done to be asserted
	while ((GPIOB->IDR & (0x0080)) == 0);

	// De-assert Enable
	GPIOB->ODR &= ODR_data;

	// Wait for Done to be de-asserted
	while ((GPIOB->IDR & (0x0080)) != 0);
}


void myTIM2_Init() {

	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;

	/* Begin timer pulsing */
	TIM2->CR1 |= TIM_CR1_CEN;
}


void myTIM3_Init(void) {

    /* Enable clock for TIM3 peripheral */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Configure TIM3: buffer auto­reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
    TIM3->CR1 = ((uint16_t)0x008C);

    /* Set clock prescaler value: 48MHz/(47999+1) = 1 KHz */
    TIM3->PSC = myTIM3_PRESCALER;

    /* Default auto­reloaded delay: 100 ms */
    TIM3->ARR = 100;

    /* Update timer registers */
    TIM3->EGR |= 0x0001;
}


void myEXTI_Init() {

	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI1);

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[2], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler() {

	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler() {

	// Declare/initialize your local variables here...
	uint32_t count;
	uint32_t timerTriggered;

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0) {

	timerTriggered = (TIM2->CR1 & TIM_CR1_CEN);

		if (timerTriggered) {

			// Stop the timer and retrieve the value from CNT
			EXTI->IMR &= ~(EXTI_IMR_MR2);
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			count = TIM2->CNT;

			freq = (double)SystemCoreClock / (double)count;
			period_us = 1000000 / freq;

			//trace_printf("Period: %d Seconds \n", (unsigned int)period_s);
			//trace_printf("Period: %d Microseconds \n", (unsigned int)period_us);
			//trace_printf("Frequency: %d Hz\n", (unsigned int)freq);

			EXTI->IMR |= EXTI_IMR_MR2;
		} else {
			TIM2->CNT= ((uint32_t)0x0000);
			TIM2->CR1 |= TIM_CR1_CEN;
		}
	EXTI->PR |= EXTI_PR_PR1;
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
