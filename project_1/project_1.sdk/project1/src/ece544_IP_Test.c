/**
*
* @file ece544ip_test.c
*
* @author Roy Kravitz (roy.kravitz@pdx.edu)
* @copyright Portland State University, 2016-2020
*
* This file implements a test program for the Nexys4IO and Digilent Pmod peripherals
* used in ECE 544. The peripherals provides access to the pushbuttons
* and slide switches, the LEDs, the RGB LEDs (only Nexys A7), and the Seven Segment display
* on the Digilent Nexys A7 and Basys 3 boards and the PmodOLEDrgb (94 x 64 RGB graphics display)
* and the PmodENC (rotary encoder + slide switch + pushbutton).
*
* The test is basic but covers all of the API functions:
*	o initialize the Nexys4IO, Pmod drivers and all the other peripherals
*	o Set the LED's to different values
*	o Check that the duty cycles can be set for both RGB LEDs
*	o Write character codes to the digits of the seven segment display banks
*	o Check that all of the switches and pushbuttons on the Nexys4 can be read
*	o Performs a basic test on the rotary encoder and pmodOLEDrgb
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a	rhk	02-Jul-2016		First release of test program.  Builds on the ece544 peripheral test used
*							to check the functionality of Nexys4IO and PMod544IOR2
* 2.00a sy  14-Oct-2016		Modified the code to include different initialize function for other peripherals 
*							connected to the system.
* 3.00	rk	05-Apr-2018		Modified for Digilent PmodENC and PmodOLEDrgb.  Replaced MB_Sleep() w/ usleep.
* 4.00	rk	30-Mar-2020		Modified to support both the Nexys A7 and Basys 3 FPGA platforms
* </pre>
*
* @note
* The minimal hardware configuration for this test is a Microblaze-based system with at least 32KB of memory,
* an instance of Nexys4IO, an instance of the pmodOLEDrgb AXI slave peripheral, and instance of the pmodENC AXI
* slave peripheral, an instance of AXI GPIO, an instance of AXI timer and an instance of the AXI UARTLite 
* (used for xil_printf() console output)
*
* @note
* The driver code and test application(s) for the pmodOLDrgb and pmodENC are
* based on code provided by Digilent, Inc.
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"
#include "microblaze_sleep.h"
#include "nexys4IO.h"
#include "PmodOLEDrgb.h"
#include "PmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"

/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0


// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL		1
#define GPIO_0_OUTPUT_0_CHANNEL		2
#define GPIO_1_DEVICE_ID			XPAR_AXI_GPIO_1_DEVICE_ID
#define GPIO_1_INPUT_0_CHANNEL		1
#define GPIO_1_OUTPUT_0_CHANNEL		2

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
// Microblaze peripheral instances
uint64_t 	timestamp = 0L;
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;
XGpio		GPIOInst0;					// GPIO instance
XGpio		GPIOInst1;					// Added for Project 1
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance


// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler


volatile uint32_t			gpio_in;			// GPIO input port


/************************** Function Prototypes *****************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											// initialize system
void FIT_Handler(void);										// fixed interval timer interrupt handler
int AXI_Timer_initialize(void);

// Project 1 additions.
void blink_dp1(unsigned int *dp_cnt);
void initial_sseg_state(void);
void button_scan(	unsigned char *val, 
					unsigned char *sat, 
					unsigned char *hue, 
					unsigned int* ticks, 
					unsigned int* state, 
					unsigned int* laststate);
void update_sseg(	unsigned char *digi1, 
					unsigned char *digi2, 
					unsigned char *digi3, 
					unsigned char *digi4);
void update_duty(u16 rgb_val);
void update_oled(u16 rgb_val, unsigned char hue, unsigned char sat, unsigned char val);

void RunTest1(void);
void RunTest2(void);
void RunTest3(void);
void RunTest4(void);
void RunTest5(void);

/************************** MAIN PROGRAM ************************************/
int main(void)
{
    init_platform();

	uint32_t sts;
	uint32_t state, laststate;

	sts = do_init();
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	microblaze_enable_interrupts();

	xil_printf("ECE 544 Nexys4io test program\n\r");
	xil_printf("By Roy Kravitz. 31-Mar-2020\n\n\r");


	RunTest4();
	
	// clear the displays and power down the pmodOLEDrbg
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_end(&pmodOLEDrgb_inst);
	
	// cleanup and exit
    cleanup_platform();
    exit(0);
}


/**************************** HELPER FUNCTIONS ******************************/

/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/

int	 do_init(void)
{
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// Added for Project 1.
	status = XGpio_Initialize(&GPIOInst1, GPIO_1_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO0 channel 1 is an 8-bit input port.
	// GPIO0 channel 2 is an 8-bit output port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);
	// Added for project 1.
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL, 0xFF);


	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}
	
	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}
/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	uint32_t		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);																// Initialize Counter.
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);																		// Selftest
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;				// Control or Status Register Mask
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);														// Set Timer Control Status Reg

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 2498);																	// Default count = 24998. (modified)
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion, 
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}
	
  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;
	
	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;
	
  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*       
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;
  
  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;
    
    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];
  
  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}

/****************************************************************************/
/**
* Test 4 - Test the PmodENC and PmodOLEDrgb
*
* The rotary encoder portion of this test is taken from the Digilent PmodENC driver example
*
* Performs some basic tests on the PmodENC and PmodOLEDrgb.  Includes the following tests
* 	1.	check the rotary encoder by displaying the rotary encoder
* 		count in decimal and hex on the LCD display.  Rotate the knob
* 		to change the values up or down.  The pushbuttons can be used
* 		as follows:
* 			o 	press the rotary encoder pushbutton to exit
* 			o 	press BtnUp to clear the count
*
*	6.	display the string "357#&CFsw" on the LCD display.  These values
* 		were chosen to check that the bit order is correct.  The screen will
* 		clear in about 5 seconds.
* 	7.	display "Looks Good" on the display.  The screen will clear
* 		in about 5 seconds.
*
*
* @param	*NONE*
*
* @return	*NONE*
*
*****************************************************************************/
#define DP_BLINK_PERIOD 5000
#define DP_1_MASK 0x2000000
#define SWITCH123_MASK 0xD
#define SWITCH23_MASK 0xC
#define SW0_MASK	0x1

void RunTest4(void){

		// CoLoR WhEeL 
		unsigned char 	hue 					= 0,  
						sat 					= 0,  
						val 					= 0,
						digi1 					= 0,
						digi2 					= 0,
						digi3 					= 0,
						digi4 					= 0;

		// Software PWDET Variables
		unsigned int 	high_count				= 0, 
						low_count				= 0, 
						count 					= 0, 
						dp_cnt 					= 0,
						gpio_in_reg				= 0, 
						dc						= 0,
						last_dc					= 0,
						channel_mask			= 0,
						shftamnt				= 0,
						gpio_in_signal			= 0,
						NX4IO_duty_cycle		= 0,
						rgb_state				= 0,
						last_rgb_state			= 0,
						switch_value 			= 0,
						sw_state				= 0,
						last_sw_state 			= 0,
						hardpwdet_value	 		= 0,
						last_hardpwdet_value 	= 0,
						state					= 0,
						laststate				= 0,
						ticks 					= 0;

	u16					rgb_val					= 0;

	bool 				edge_trigger 			= false;


	// Initial State
	// turn off all of the decimal points
	initial_sseg_state();

	// get the previous state
	laststate = ENC_getState(&pmodENC_inst);

	while(1) {

		// get states.
		state = ENC_getState(&pmodENC_inst);
		rgb_state = (state<<24) | (hue << 8) | (val << 16) | (sat) ;
		sw_state = NX4IO_getSwitches() & SWITCH123_MASK;

		// check if the rotary encoder pushbutton is pressed
		// exit the loop if either one is pressed.
		if (ENC_buttonPressed(state) && !ENC_buttonPressed(laststate)) break;

		blink_dp1(&dp_cnt);		// Blink the 1st decimal point.

		initial_sseg_state();	// Initialize the SSEG display.

		button_scan(&val, &sat, &hue, &ticks, &state, &laststate);	// Check for button pressed.

		rgb_val = OLEDrgb_BuildHSV(hue, sat, val);	
	
		// Display the channel selected on LEDs[3:2] and [0].
		switch_value = (NX4IO_getSwitches() & SWITCH123_MASK);
			
		// Software PWDET
		// Display the PWDET value on the SSEG from channel selected on LEDs[3:2].
		// assign gpio_in = {5'b00000, w_RGB1_Red, w_RGB1_Blue, w_RGB1_Green}; //wire	[7:0]	    gpio_in;
		switch((switch_value & SWITCH23_MASK) >> 2){
			case 0: channel_mask = 0x4; shftamnt = 2; NX4IO_duty_cycle = ((((rgb_val>>11) & 0x1F)*100)/255); break;	//Red signal
			case 1: channel_mask = 0x1; shftamnt = 0; NX4IO_duty_cycle = ((((rgb_val>>5) & 0x3F)*100)/255); break;	// Green Signal
			case 2: channel_mask = 0x2; shftamnt = 1; NX4IO_duty_cycle = (((rgb_val & 0x1F)*100)/255); break;	// Blue Signal
			default: break;
		}

		// Read the GPIO port to read back the generated PWM signal for RGB led's
		gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);
		hardpwdet_value = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL);

		// Extract 1-bit signal from gpio_in.
		gpio_in_signal = (gpio_in & channel_mask) >> shftamnt;

		// Test for rising/falling edge.
		edge_trigger = (gpio_in_signal != gpio_in_reg) ? true : false;

		// If edge is present, assign count. 
		if(edge_trigger){
			if(gpio_in_signal){		// Rising edge.
				low_count = count;
				
				// Calculate duty cycle (limit ceiling).
				dc = (high_count * 100) / (high_count + low_count);
				if(dc == 100) dc = 99;

				// Reset for next period. 
				count = 1;
			}
			else
			{
				high_count = count;	// Falling edge.
				count = 1;
			}
			
		}


		// Calculated PW.
		if(NX4IO_duty_cycle > 99){
			digi2 = 0x9;
			digi1 = 0x9;
		}
		else{
			digi2 = (NX4IO_duty_cycle / 10);
			digi1 = NX4IO_duty_cycle % 10;
		}

		// HW-SW switch.
		if(NX4IO_getSwitches() & SW0_MASK){
			// Hardware detect
			digi4 = hardpwdet_value / 10;
			digi3 = hardpwdet_value % 10;

		}
		else{// Software detect
			digi4 = dc / 10;
			digi3 = dc % 10;
		} 

		// Quality assurance.
		if(digi2 > 10) digi2 = 0x9;

		// Clock in the value for future comparison.
		gpio_in_reg = gpio_in_signal;

		// Increment counter (Always running).
		count++;

		// Update LEDs with switch values.
		NX4IO_setLEDs(switch_value);

		// Set up the display output
		if((rgb_state != last_rgb_state)){

			
			 update_oled(((((rgb_val >> 5)&0x7FF)) |
			 				((rgb_val & 0x1F)<<11)), hue, sat, 255);			
			
			//update_oled(rgb_val, hue, sat, 255);

			update_sseg(&digi1, &digi2, &digi3, &digi4);

			update_duty(rgb_val);

			// Update states.
			last_dc = dc;
			last_sw_state = sw_state;
			last_rgb_state = rgb_state;
			laststate = state;

			usleep(1000);
		} // rotary button has been pressed - exit the loop
	}
	return;
}

/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO port which reads back the hardware generated PWM wave for the RGB Leds
*
* @note
* ECE 544 students - When you implement your software solution for pulse width detection in
* Project 1 this could be a reasonable place to do that processing.
 *****************************************************************************/
#define SW0_MASK 0x1

void FIT_Handler(void)
{
	static int dp_cnt2 = 0;

	// Decimal Point blink routine
	if(dp_cnt2 == DP_BLINK_PERIOD){
		dp_cnt2 = 0;
		if((DP_1_MASK ^ NX4IO_SSEG_getSSEG_DATA(SSEGLO))>>25) NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, 1);
			else			NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, 0);
	}
	dp_cnt2++;

}

void blink_dp1(unsigned int *dp_cnt){
	// Decimal Point blink routine
		if(*dp_cnt == DP_BLINK_PERIOD){
			*dp_cnt = 0;
			if((DP_1_MASK ^ NX4IO_SSEG_getSSEG_DATA(SSEGLO))>>25) NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, 1);
				else			NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, 0);
		}
		*dp_cnt = *dp_cnt + 1;
}

void initial_sseg_state(void){
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT7, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT6, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT5, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT4, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT3, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT2, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT0, false);
}

void button_scan(unsigned char *val, unsigned char *sat, unsigned char *hue, unsigned int* ticks, unsigned int* state, unsigned int* laststate){
		// check BTNU and clear count if it's pressed
		// update the count if it is not
		if (NX4IO_isPressed(BTNU))
		{
			if(*val <= 255) *val = (*val << 1) | 0x1;
			else *val = *val;
			*val = *val & 0xFF;
			while(NX4IO_isPressed(BTNU)); //1 second wait.
			//while(NX4IO_isPressed(BTNU));
		}
		if (NX4IO_isPressed(BTND))
		{
			if(*val >= 1) *val = (*val>> 1) & 0x7F;
			else *val = *val;
			*val = *val & 0xFF;
			while(NX4IO_isPressed(BTND)); //1 second wait.
			//while(NX4IO_isPressed(BTND));

		}
		if (NX4IO_isPressed(BTNL))
		{
			if(*sat >=1) *sat = (*sat >> 1)  & 0x7F;
			else *sat = *sat;
			*sat = *sat & 0xFF;
			while(NX4IO_isPressed(BTNL)); //1 second wait.
			//while(NX4IO_isPressed(BTNL));
		}
		if (NX4IO_isPressed(BTNR))
		{
			if(*sat <= 255) *sat = (*sat << 1) | 0x1;
			else *sat = *sat;
			*sat = *sat & 0xFF;
			while(NX4IO_isPressed(BTNR)); //1 second wait.
			//while(NX4IO_isPressed(BTNR));
		}
		else
		{
			// BTNU is not pressed so increment count
			*ticks += 5*(ENC_getRotation(*state, *laststate));
		}
		
		// Guard the tick value.
		if(*val > 255) *val = 255;
		if(*sat > 255) *sat = 255;
		if(*ticks > 255) *ticks = 0;
		else if(*ticks < 0) *ticks = 255;
		else *hue = *ticks;
		*hue = *hue & 0xFF;

}

void update_oled(u16 rgb_val, unsigned char hue, unsigned char sat, unsigned char val){
		OLEDrgb_Clear(&pmodOLEDrgb_inst);
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst, rgb_val); // Not correct args names
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"H:");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"S:");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 4);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"V:");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 2);
		PMDIO_putnum(&pmodOLEDrgb_inst, hue, 10);
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);
		PMDIO_putnum(&pmodOLEDrgb_inst, sat, 10);
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 4);
		PMDIO_putnum(&pmodOLEDrgb_inst, val, 10);
}

void update_sseg(	unsigned char *digi1, 
					unsigned char *digi2, 
					unsigned char *digi3, 
					unsigned char *digi4){
		// FInally write to SSEG:
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, *digi4);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT4, *digi3);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, *digi2);
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT2, *digi1);	
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT1, 0x1E);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, 0x1E);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT3, 0x1E);
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT2, 0x1E);
}

void update_duty(u16 rgb_val){

		NX4IO_RGBLED_setDutyCycle(RGB1, ((rgb_val>>11) & 0x1F), ((rgb_val>>5) & 0x3F), (rgb_val & 0x1F));
		NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
}