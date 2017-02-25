/*************************************************
 Jonathan Coley
 University of North Texas 
 11/20/2014 
 
 Input 5 ADC values using PD0, PD1, PD2, PD3, and
 PE1 from from a voltage divider circuit using flex sensors
 in the fingers of a glove to control 5 PWM signals using
 PA6, PF0, PF1, PF2, and PF3 that will control servos
 attatched to the tendons of a robotic arm.

			ADC				PWM
thumb		PD0				PA6
index		PD1				PF0
middle		PD2				PF1
ring		PD3				PF2
pinky		PE1				PF3

 To calibrate the fingers according to your circuit open a command
 window and run the program along with the ciruit. Write what values that
 correspond to unbent and bent characteristics of the flex sensor and
 place that in the max or min value of the global variable. ThE mapping 
 should do the rest of the work for you. These global variables are intialized 
 to zero just below but you need to change them in the map function. From
 there you can see my circuits values of the ADC max and min from each flex
 sensor placed on the glove fingers.
 

  The code can also be tested by changing the color of the LED flashing on the 
  board by entering r, g, or b in the command window. To use this test just uncomment
  the while loop with the switch and case statement.
  
 	 Also, this code can also display the actual voltage at the ADC. Just uncomment the lines
 	 that look like //pinky = 3.3*pinky_data/4095 within the Timer1 Iterupt
  
 NOTE: DO NOT EXCEED 3.3V into the ADC, ALSO DO NOT TRY TO POWER SERVOS WITH THE LAUNCHPAD

	This code was implemented on TI's TivaLaunchpad
	Datasheet of the LaunchPad can be found here:
	http://www.ti.com/lit/ds/spms376e/spms376e.pdf
	
 PWM code started with example code by
 lawrence_jeff found here:
 http://forum.stellarisiti.com/topic/707-using-hardware-pwm-on-tiva-launchpad/

 ADC code started sample code given in class at
 the University of North Texas By Dr. Xinrong Li, xinrong@UNT.EDU, Sept. 10, 2014

 The programs altered by Jonathan Coley November 4, 2016
 *************************************************/
#include <stdint.h> // Variable definitions for the C99 standard.
#include <stdio.h> // Input and output facilities for the C99 standard.

#include <stdlib.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h" // Definitions for the interrupt and register assignments.
#include "inc/hw_memmap.h" // Memory map definitions of the Tiva C Series device.
#include "inc/hw_types.h" // Definitions of common types and macros.
#include "inc/hw_gpio.h"

#include "driverlib/sysctl.h"
#include "driverlib/sysctl.h" // Definitions and macros for System Control API of DriverLib.
#include "driverlib/interrupt.h" // Defines and macros for NVIC Controller API of DriverLib.
#include "driverlib/gpio.h" // Definitions and macros for GPIO API of DriverLib.
#include "driverlib/timer.h" // Defines and macros for Timer API of DriverLib.
#include "driverlib/pin_map.h" //Mapping of peripherals to pins for all parts.
#include "driverlib/uart.h" // Definitions and macros for UART API of DriverLib.
#include "driverlib/adc.h" // Definitions for ADC API of DriverLib.
#include "driverlib/fpu.h" // Prototypes for the FPU manipulation routines.
#include "utils/uartstdio.h" // Prototypes for the UART console functions.
							 // Needs to add "utils/uartstdio.c" through a relative link.
#include "driverlib/pwm.h"

#define TIMER0_FREQ   2  // Freqency in Hz
#define TIMER1_FREQ    10 // Freqency in Hz
#define UART0_BAUDRATE    115200 // UART baudrate in bps

#define ADC0_SEQ_NUM 0 // ADC Sample Sequence Number

//#define RED_LED    GPIO_PIN_1
//#define BLUE_LED    GPIO_PIN_2
//#define GREEN_LED    GPIO_PIN_3


#define DISP_TEXT_LINE_NUM    4
#define VOLT_STR_LEN    50


// Functions for general purpose program functionality
void pwm_map(void);
void init_timer(void);
void Timer0_ISR(void);
void Timer1_ISR(void);

//Functions for ADC functionality (UART reads and prints ADC values)
void init_ADC(void);
void init_UART(void);
extern void UARTStdioIntHandler(void);

//Functions for PWM functionality
void init_pwm(void);
void unlock_pin(void);

//Holds the value of the system Clock
uint32_t sys_clock;

// Sets the initial value of the on board LED to Red
//uint8_t cur_LED = RED_LED;

// Initial instruction prompt sent to the user via UART
const char *disp_text[DISP_TEXT_LINE_NUM] = {
		"\n",
		"UART and LED Demo\n",
		"H: help, R: red, G: green, B: blue.\n",
		"> " };


// X_data holds the value from the FIFO register from the ADC
uint32_t   thumb_data = 0, index_data = 0, middle_data = 0, ring_data = 0, pinky_data = 0;

//Functions and variables for mapping the ADC signal
uint32_t thumb_pwm = 0, prev_thumb = 0, thumb_buffer = 0;
uint32_t index_pwm = 0, prev_index = 0, index_buffer = 0;
uint32_t middle_pwm = 0, prev_middle = 0, middle_buffer = 0;
uint32_t ring_pwm = 0, prev_ring = 0, ring_buffer = 0;
uint32_t pinky_pwm = 0, prev_pinky = 0, pinky_buffer = 0;

// Float variabls that hold the value of the voltage drop
// that will be displayed to the user via UART
float 		thumb, index, middle, ring, pinky;
char volt_str1[VOLT_STR_LEN], volt_str2[VOLT_STR_LEN], volt_str3[VOLT_STR_LEN], volt_str4[VOLT_STR_LEN], volt_str5[VOLT_STR_LEN];

int main(void)
{
	uint32_t i;

	// Configure system clock at 40 MHz.
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	sys_clock = SysCtlClockGet();

	// Enable the floating-point unit (FPU).
	FPUEnable();
	// Configure FPU to perform lazy stacking of the floating-point state.
	FPULazyStackingEnable();

	unlock_pin();
	init_pwm();
	init_ADC();
	init_UART();
	init_timer();

	// Enable the processor to respond to interrupts.
	IntMasterEnable();

	// Start the timer by enabling operation of the timer module.
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);
	// Initial display on terminal.
	for(i=0; i<DISP_TEXT_LINE_NUM; i++)
		UARTprintf(disp_text[i]);

	/*while(1) {
		// Read user inputs from UART if available.
		if(UARTRxBytesAvail())
	        user_cmd = UARTgetc();
		else
					user_cmd = 0;


		switch(user_cmd){
		case '\r':
		case ' ':
		case 'H':
		case 'h':
			for(i=0; i<DISP_TEXT_LINE_NUM; i++)
				UARTprintf(disp_text[i]);
			break;
		case 'R':
		case 'r':
			cur_LED = RED_LED;
			UARTprintf("\n> ");
			break;
		case 'B':
		case 'b':
			cur_LED = BLUE_LED;
			UARTprintf("\n> ");
			break;
		case 'G':
		case 'g':
			cur_LED = GREEN_LED;
			UARTprintf("\n> ");
			break;


		}//switch

	}//while */
}//main
void unlock_pin(void)
{
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Unlock the Pin PF0 and Set the Commit Bit
    // See datasheet table 10-1 for explanation of
    // why this pin needs unlocking
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)   |= 0x01;
}

void init_pwm(void)
{
	//Configure PWM Clock to match system
   SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

   // Enable the peripherals used by this program.

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins


    //Configure PF0,PF1,PF2,PF3, and PA6 Pins as PWM
    //See table 20-1 for these assignments

    GPIOPinConfigure(GPIO_PA6_M1PWM2);
    GPIOPinConfigure(GPIO_PF0_M1PWM4);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);

    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
   GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See table 20-1 for these assignments
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //These settings are specifically designed to run servo motors
    //which expect 20mS period with between 1ms and 2ms high time
    //
    //System clock is 16MHz with PWM divider of 64
    // 16000000/64 = 250000/50 = 5000  ### 1S/50 = 20mS thats where divisor comes from
    unsigned long period = 5000;
    // Set high time to 2mS


    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, period);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, period);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, period);


    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
   // PWM_OUT_4_BIT
    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT |PWM_OUT_4_BIT| PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
}

void Timer1_ISR(void)
{

	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	ADCSequenceDisable(ADC0_BASE, ADC0_SEQ_NUM);

	//Channel 7 is Thumb (PD0) PWM2(PA6)CHANNEL 2

				ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
				ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH7 |ADC_CTL_IE|ADC_CTL_END);
				ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);

				ADCProcessorTrigger(ADC0_BASE, ADC0_SEQ_NUM);

				while(!ADCIntStatus(ADC0_BASE, ADC0_SEQ_NUM, false)) {
					}
					ADCIntClear(ADC0_BASE, ADC0_SEQ_NUM);
					ADCSequenceDataGet(ADC0_BASE, ADC0_SEQ_NUM, &thumb_data);

					//thumb = 3.3*thumb_data/4095;
					thumb = thumb_data;
					thumb_buffer = thumb_data;
					if ( thumb > 2){
						snprintf(volt_str1, VOLT_STR_LEN, "%.2f = Thumb Sensor is unbent",thumb);
									}
					else {
						snprintf(volt_str1, VOLT_STR_LEN, "%.2f = Thumb Sensor is bent",thumb);

					}//

																// Print current temperature to UART0
					UARTprintf("\nVolt = %s> ", volt_str1);
	//End of Thumb (PD0)

	// INDEX PD1 PWM4(PF0)CHANNEL 4
			ADCSequenceDisable(ADC0_BASE, ADC0_SEQ_NUM);
				ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
				ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH6 |ADC_CTL_IE|ADC_CTL_END);
				ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);

				ADCProcessorTrigger(ADC0_BASE, ADC0_SEQ_NUM);

								while(!ADCIntStatus(ADC0_BASE, ADC0_SEQ_NUM, false)) {
								}
								ADCIntClear(ADC0_BASE, ADC0_SEQ_NUM);
					ADCSequenceDataGet(ADC0_BASE, ADC0_SEQ_NUM, &index_data);

					//index = 3.3*index_data/4095;
					index = index_data;

					index_buffer = index_data;

					if ( index > 2){
						snprintf(volt_str2, VOLT_STR_LEN, "%.2f = Index Sensor is unbent",index);
									}
					else {

						snprintf(volt_str2, VOLT_STR_LEN, "%.2f = Index Sensor is bent",index);
						}

											// Print current temperature to UART0
											UARTprintf("\nVolt = %s> ", volt_str2);
	//End of index (PD1)


	//Channel 5 is Middle (PD2) PWM(PA7)CHANNEL 3

			ADCSequenceDisable(ADC0_BASE, ADC0_SEQ_NUM);
				ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
				ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH5 |ADC_CTL_IE|ADC_CTL_END);
				ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);

				ADCProcessorTrigger(ADC0_BASE, ADC0_SEQ_NUM);

								while(!ADCIntStatus(ADC0_BASE, ADC0_SEQ_NUM, false)) {
								}
								ADCIntClear(ADC0_BASE, ADC0_SEQ_NUM);
					ADCSequenceDataGet(ADC0_BASE, ADC0_SEQ_NUM, &middle_data);

					//middle = 3.3*middle_data/4095;
					middle = middle_data;

					middle_buffer = middle_data;
					if ( middle > 2){
						snprintf(volt_str3, VOLT_STR_LEN, "%.2f =Middle Sensor is unbent",middle);
									}
					else {
						snprintf(volt_str3, VOLT_STR_LEN, "%.2f =Middle Sensor is bent",middle);
						}

						// Print current temperature to UART0
						UARTprintf("\nVolt = %s> ", volt_str3);
	//End of Middle (PD2)

	//Channel 4 is Ring (PD3) PWM(PB7)CHANNEL 1

			ADCSequenceDisable(ADC0_BASE, ADC0_SEQ_NUM);
				ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
				ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH4 |ADC_CTL_IE|ADC_CTL_END);
				ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);

				ADCProcessorTrigger(ADC0_BASE, ADC0_SEQ_NUM);

								while(!ADCIntStatus(ADC0_BASE, ADC0_SEQ_NUM, false)) {
								}
								ADCIntClear(ADC0_BASE, ADC0_SEQ_NUM);
					ADCSequenceDataGet(ADC0_BASE, ADC0_SEQ_NUM, &ring_data);

					//ring = 3.3*ring_data/4095;
					ring = ring_data;

					ring_buffer = ring_data;
					if ( ring > 2){
						snprintf(volt_str4, VOLT_STR_LEN, "%.2f =Ring Sensor is unbent",ring);
									}
					else {
						snprintf(volt_str4, VOLT_STR_LEN, "%.2f =Ring Sensor is bent",ring);
						}

						// Print current temperature to UART0
						UARTprintf("\nVolt = %s> ", volt_str4);
	//End of ring (PD3)

	//Channel 2 is Pinky (PE1) PWM(PB6)CHANNEL 0

			ADCSequenceDisable(ADC0_BASE, ADC0_SEQ_NUM);
				ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
				ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH2 |ADC_CTL_IE|ADC_CTL_END);
				ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);

				ADCProcessorTrigger(ADC0_BASE, ADC0_SEQ_NUM);

								while(!ADCIntStatus(ADC0_BASE, ADC0_SEQ_NUM, false)) {
								}
								ADCIntClear(ADC0_BASE, ADC0_SEQ_NUM);
					ADCSequenceDataGet(ADC0_BASE, ADC0_SEQ_NUM, &pinky_data);

					//pinky = 3.3*pinky_data/4095;
					pinky = pinky_data;

					pinky_buffer = pinky_data;
					if ( pinky > 1.8){
						snprintf(volt_str5, VOLT_STR_LEN, "%.2f =Pinky Sensor is unbent",pinky);
										}
					else {
						snprintf(volt_str5, VOLT_STR_LEN, "%.2f =Pinky Sensor is bent",pinky);
						}

											// Print current temperature to UART0
											UARTprintf("\nVolt = %s> ", volt_str5);
											UARTprintf("\n");
	//End of pinky (PE1)


			pwm_map();


}
void pwm_map(void)
{
// Max and min values for ADC and PWM(0' - 180')
uint32_t thumb_adc_max = 2300;
uint32_t thumb_adc_min = 1800;
uint32_t thumb_pwm_max = 1100;
uint32_t thumb_pwm_min = 250;

uint32_t index_adc_max = 2250;
uint32_t index_adc_min = 1545;
uint32_t index_pwm_max = 1100;
uint32_t index_pwm_min = 250;

uint32_t middle_adc_max = 2600;
uint32_t middle_adc_min = 2200;
uint32_t middle_pwm_max = 1100;
uint32_t middle_pwm_min = 250;

uint32_t ring_adc_max = 2285;
uint32_t ring_adc_min = 1770;
uint32_t ring_pwm_max = 1100;
uint32_t ring_pwm_min = 250;

uint32_t pinky_adc_max = 2600;
uint32_t pinky_adc_min = 2000;
uint32_t pinky_pwm_max = 1100;
uint32_t pinky_pwm_min = 250;

uint32_t thumb_calib	= abs(4095-thumb_adc_max - thumb_adc_min);
uint32_t index_calib	= abs(4095-index_adc_max- index_adc_min);
uint32_t middle_calib	= abs(4095-middle_adc_max- middle_adc_min);
uint32_t ring_calib		= abs(4095-ring_adc_max- ring_adc_min);
uint32_t pinky_calib 	= abs(4095-pinky_adc_max- pinky_adc_min);



//Thumb mapping
	// 		MAX			MIN
	//ADC	2595		1595
	//PWM	1100		250
	if (thumb_buffer > thumb_adc_max)
		{thumb_buffer = thumb_adc_max;}

	if (thumb_buffer < thumb_adc_min)
		{thumb_buffer = thumb_adc_min;}

	if( abs(thumb_buffer - prev_thumb) < 25)
		{thumb_buffer = prev_thumb;}
	else
		{prev_thumb = thumb_buffer;}

	thumb_pwm = 4095-thumb_buffer;
	thumb_pwm = ((thumb_pwm + thumb_calib) - thumb_adc_min) * (thumb_pwm_max - thumb_pwm_min) / (thumb_adc_max - thumb_adc_min) + thumb_pwm_min;

//index mapping
	// 		MAX			MIN
	//ADC	2595		1595
	//PWM	1100		250
	if (index_buffer > index_adc_max)
		{index_buffer = index_adc_max;}

	if (index_buffer < index_adc_min)
		{index_buffer = index_adc_min;}

	if( abs(index_buffer - prev_index) < 15)
		{index_buffer = prev_index;}
	else
		{prev_index = index_buffer;}

	index_pwm = 4095-index_buffer;
	index_pwm = ((index_pwm - index_calib) - index_adc_min) * (index_pwm_max - index_pwm_min) / (index_adc_max - index_adc_min) + index_pwm_min;

//Middle mapping
	// 		MAX			MIN
	//ADC	2595		1595
	//PWM	1100		250
	if (middle_buffer > middle_adc_max)
	{middle_buffer = middle_adc_max;}

	if (middle_buffer < middle_adc_min)
		{middle_buffer = middle_adc_min;}

	if( abs(middle_buffer - prev_middle) < 15)
		{middle_buffer = prev_middle;}
	else
		{prev_middle = middle_buffer;}

	middle_pwm = 4095-middle_buffer;
	middle_pwm = ((middle_pwm + middle_calib) - middle_adc_min) * (middle_pwm_max - middle_pwm_min) / (middle_adc_max - middle_adc_min) + middle_pwm_min;

//ring mapping
	// 		MAX			MIN
	//ADC	2595		1595
	//PWM	1100		250
	if (ring_buffer > ring_adc_max)
	{ring_buffer = ring_adc_max;}

	if (ring_buffer < ring_adc_min)
		{ring_buffer = ring_adc_min;}

	if( abs(ring_buffer - prev_ring) < 15)
		{ring_buffer = prev_ring;}
	else
		{prev_ring = ring_buffer;}

	ring_pwm = 4095-ring_buffer;
	ring_pwm = ((ring_pwm + ring_calib) - ring_adc_min) * (ring_pwm_max - ring_pwm_min) / (ring_adc_max - ring_adc_min) + ring_pwm_min;

	//ring mapping
		// 		MAX			MIN
		//ADC	2595		1595
		//PWM	1100		250
		if (pinky_buffer > pinky_adc_max)
		{pinky_buffer = pinky_adc_max;}

		if (pinky_buffer < pinky_adc_min)
			{pinky_buffer = pinky_adc_min;}

		if( abs(pinky_buffer - prev_pinky) < 15)
			{pinky_buffer = prev_pinky;}
		else
			{prev_pinky = pinky_buffer;}

		pinky_pwm = 4095-pinky_buffer;
		pinky_pwm = ((pinky_pwm + pinky_calib) - pinky_adc_min) * (pinky_pwm_max - pinky_pwm_min) / (pinky_adc_max - pinky_adc_min) + pinky_pwm_min;

			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, thumb_pwm);
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, index_pwm);
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, middle_pwm);
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ring_pwm);
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, pinky_pwm);
}


void init_timer(void)
{
	// Enable and configure timer peripheral.
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

		// Configure Timer0 as a 32-bit timer in periodic mode.
		TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
		// Initialize timer load register.
		TimerLoadSet(TIMER0_BASE, TIMER_A, sys_clock/TIMER0_FREQ -1);

		// Configure Timer1 as a 32-bit timer in periodic mode.
		TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
		// Initialize timer load register.
		TimerLoadSet(TIMER1_BASE, TIMER_A, sys_clock/TIMER1_FREQ -1);


		// Registers a function to be called when the interrupt occurs.
		IntRegister(INT_TIMER0A, Timer0_ISR);
		// The specified interrupt is enabled in the interrupt controller.
		IntEnable(INT_TIMER0A);
		// Enable the indicated timer interrupt source.
		TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

		// Registers a function to be called when the interrupt occurs.
		IntRegister(INT_TIMER1A, Timer1_ISR);
		// The specified interrupt is enabled in the interrupt controller.
		IntEnable(INT_TIMER1A);
		// Enable the indicated timer interrupt source.
		TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}


void init_UART(void)
{
	// Enable and configure UART0 for debugging printouts.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_UART0, UARTStdioIntHandler);
	UARTStdioConfig(0, UART0_BAUDRATE, sys_clock);
}


void init_ADC(void)
{
	// Enable and configure ADC0. Sample from PD0/AIN7/BY
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);


	ADCSequenceConfigure(ADC0_BASE, ADC0_SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH7);
	ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH6);
	ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH5);
	ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC0_BASE, ADC0_SEQ_NUM, 0, ADC_CTL_CH2 |ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, ADC0_SEQ_NUM);


	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);


}


// Timer0 interrupt service routine
void Timer0_ISR(void)
{
	// Clear the timer interrupt.
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Blink LED. Read the current state of GPIO pins and write back the opposite state.
	//if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)) {
	//	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	//}
	//else {
	//	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, cur_LED);
	//}
}

