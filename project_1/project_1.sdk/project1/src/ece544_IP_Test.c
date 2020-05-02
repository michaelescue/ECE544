/**
*
* @file ece544ip_test.c
*
* @author Roy Kravitz (roy.kravitz@pdx.edu), Modified by Michael Escue (mescue@pdx.edu)
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
* 5.00  me  01-Apr-2020		Repurposed for use in ECE544 Project1, Spring 2020.
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

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/
#define DP_BLINK_PERIOD 50000	// Used for Decimal Point 1 Blinking Intervals
#define DP_1_MASK 0x2000000		// Used for Decimal Point 1 Masking of the SSEGLO_data reg.
#define SWITCH023_MASK 0xD		// Used to mask the switch[3:2] and sw[0] values from the BTNSW_IN reg
#define SWITCH23_MASK 0xC		// Used to mask the switch[3:2] values from the BTNSW_IN reg
#define SW0_MASK	0x1			// Used to mask the switch[0] value from the BTNSW_IN reg

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

unsigned int gpio_in_reg 			= 0;
unsigned int high_count				= 0;
unsigned int low_count				= 0;
unsigned int hcount					= 0;
unsigned int lcount					= 0;
unsigned int dc						= 0;
unsigned int channel_mask			= 0;
unsigned int shftamnt				= 0;


/************************** Function Prototypes *****************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											// initialize system
void FIT_Handler(void);										// fixed interval timer interrupt handler
int AXI_Timer_initialize(void);

// Project 1 additions.
void blink_dp1(unsigned int *dp_cnt);
void initial_sseg_state(void);
void update_duty(u16 rgb_val);
int my_hsv_builder(unsigned char *hue, unsigned char *sat, unsigned char * val);
void swdetect(void);
void myfunc(void);
void exit_func(void);
void update_oled(		u16 *oled_rgb_val, 
						unsigned char *hue, 
						unsigned char *sat, 
						unsigned char *val,
						u16 *rgb_val
						);
void button_scan(		unsigned char *val, 
						unsigned char *sat, 
						unsigned char *hue
						);
void update_sseg(		unsigned char *digi1, 
						unsigned char *digi2, 
						unsigned char *digi3, 
						unsigned char *digi4
						);
void calculate_dc(		unsigned int *NX4IO_duty_cycle, 
						unsigned char *digi1, 
						unsigned char *digi2,
						unsigned char *digi3,
						unsigned char *digi4,
						unsigned int *pwdet_select,
						volatile u8  *hardpwdet_value
						);
void channel_select(	unsigned int *switch_value,
						unsigned int *NX4IO_duty_cycle,
						unsigned int *rgb_val
						);

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

	myfunc(); // Implementation for Project 1 ECE 544 Spring 2020.
	
	// clear the displays and power down the pmodOLEDrbg
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_end(&pmodOLEDrgb_inst);

	exit_func();
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
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);																	// Default count = 24998. (modified)
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

/**
 * @brief "myfunc" contains the code required to meet specifications for Project 1 (mostly). All valus are automatic and passed by reference to called functions.
 *
 * @param hue The hue value modified by the ENC rotary.
 * @param sat The sat value modified by the left and right buttons.
 * @param val The val value modified by the up and down buttons.
 * @param digi1 The value displayed on Digit one of the lower SSEG bank.
 * @param digi2 The value displayed on Digit two of the lower SSEG bank.
 * @param digi3 The value displayed on Digit three of the lower SSEG bank.
 * @param digi4 The value displayed on Digit four of the lower SSEG bank.
 * @param dp_cnt The counter value controlling the blink rate of Decimal Point 1.
 * @param shftamnt The shift amount used depending on sw[3:2] value which selects the RGBled driver signal (red, blue, or green) for Pulse Width Detection and NX4IO duty cycle calculations.
 * @param NX4IO_duty_cycle The calculated duty cycle based on 8 bit values for R, G, and B. Depends on sw[3:2] value.
 * @param switch_value The value of switches obtained from the BTNSW reg of the NX4IO core.
 * @param last_sw The previous value of switches.
 * @param state The retrieved state of the ENC Pmod.
 * @param laststate The previous retrieved state of the ENC Pmod. Used for rotation detection.
 * @param ticks The value of rotation ticks.
 * @param lastticks The previous value of rotation ticks.
 * @param pwdet_select The switch between Software and Hardware Pulse Width Detection selection based on sw[0] value.
 * @param btn_state The value of the buttons.
 * @param last_btn The previous value of the buttons.
 * @param gpio_in_signal The single bit signal representing Green, Blue, or Red drive wire from NX4IO which drives the RGBLED1. Read from GPIO0 Determined by sw[3:2] value.
 * @param brgb8 The 888 bit rgb duty cycle format based on Hue, Sat, and Val variables.
 * @param oled_rgb_val The 565 rgb duty cycle format shifted for color matching in the OLEDrgb display.
 * @param btn_changed Used as a conditional test. Directly represents/registers the value of the btns in the current state of the loop.
 * @param sw_changed Used as a conditional test to see if the switch value changed from the previous loop.
 * @param enc_changed Used as a conditional test to see if the ENC state changed from the previous loop.
 * @param ticks_chanted Used as a conditional test to see if the rotation count changed from the previous loop.
 * @param last_dc The previous value of the Software Detect Duty Cycle calculation.
 * @param dc_changed Used as a conditional test to see if the Duty Cycle changed from the previous loop.
 * @param hardpwdet_value The Duty Cycle Value calculated by the Hardware Pulse Width detection module. Read from GPIO1
 * @param rgb_val The 16 bit 565 rgb format duty cycle value based on Hue, Sat, and Val variables.
 */
void myfunc(void){
		// CoLoR WhEeL 
		unsigned char 	hue 					= 0,
						sat 					= 0,
						val 					= 0,
						digi1 					= 0,
						digi2 					= 0,
						digi3 					= 0,
						digi4 					= 0;

		// Software PWDET Variables
		unsigned int 	dp_cnt 					= 0,
						shftamnt				= 0,
						NX4IO_duty_cycle		= 0,
						switch_value 			= 0,
						last_sw					= 0,
						state					= 0,
						laststate				= 0,
						ticks 					= 0,
						lastticks				= 0,
						pwdet_select			= 0,
						btn_state 				= 0,
						last_btn				= 0,
						gpio_in_signal 			= 0,
						brgb8					= 0,
						oled_rgb_val			= 0,
						btn_changed				= 0,
						sw_changed				= 0,
						enc_changed				= 0,
						ticks_changed			= 0,
						last_dc					= 0,
						dc_changed				= 0;

	volatile u8			hardpwdet_value			= 0;

	u16					rgb_val					= 0;



	// Initial State
	// turn off all of the decimal points
	initial_sseg_state();

	// Turn off all leds excep those we care for.
	NX4IO_setLEDs(NX4IO_getSwitches() & SWITCH023_MASK);

	// Initial state of ENC pmod.
	laststate = ENC_getState(&pmodENC_inst);
	
	while(1) {

		// Next state changes

		btn_changed =  NX4IO_getBtns();

		sw_changed = last_sw != (switch_value = (NX4IO_getSwitches() & SWITCH023_MASK));

		enc_changed = laststate != (state = ENC_getState(&pmodENC_inst));

		ticks_changed = (lastticks != (ticks = (ticks + (5*ENC_getRotation(state, laststate)))));

		swdetect();

		dc_changed = (last_dc != dc);

		// Outputs

		if(enc_changed){	
			if(ENC_buttonPressed(state) && !ENC_buttonPressed(laststate)) /* ENC Button pressed */ break;

			hue = (ticks % 255) & 0xFF;	// Update hue

		}
				
		if(sw_changed){		
			high_count = 0;
			low_count = 0;

			if((switch_value & SW0_MASK) != (last_sw & SW0_MASK)){ /* sw[0] changed */
				pwdet_select = NX4IO_getSwitches() & SW0_MASK;	// Select PWDET hardware or software
			}
		}

		if(btn_changed) 
			button_scan(&val, &sat, &hue);	// Adjust values according to button push.

		if(sw_changed || btn_changed || ticks_changed || dc_changed){

			brgb8 = my_hsv_builder(&hue, &sat, &val);		// Build 8-bit RGB value for duty calc.

			channel_select(&switch_value, &NX4IO_duty_cycle, &brgb8 );	// Select an RGB channel.
		}

		if(sw_changed || btn_changed || ticks_changed || dc_changed){

			calculate_dc(&NX4IO_duty_cycle, &digi1, &digi2, &digi3, &digi4, &pwdet_select, &hardpwdet_value);
		}

		if(btn_changed || ticks_changed){
			
			rgb_val = OLEDrgb_BuildHSV(hue, sat, val);	// calculate 565 bit format RGB values.

			update_duty(rgb_val);	// Update RGBled
		}

		if(btn_changed || ticks_changed){

			oled_rgb_val = ((rgb_val >> 6) & 0x07E0) | ((rgb_val & 0x1F)<<11) | (rgb_val & 0x07E0)>>5; // Formatting for OLED

			update_oled(&oled_rgb_val, &hue, &sat, &val, &rgb_val);// Update OLED
		}

		if(sw_changed || btn_changed || ticks_changed || dc_changed)
			update_sseg(&digi1, &digi2, &digi3, &digi4); // DIgits 2, 1 (tens, ones) of calculated

		if(sw_changed) 
			NX4IO_setLEDs(switch_value);	// Update LEDS

		// Always

		blink_dp1(&dp_cnt);		// Blink the 1st decimal point.

		// Update current states

		last_dc = dc;	// Update the DC state

		last_btn = btn_state;// Update button state

		last_sw = switch_value;// Update switch state

		laststate = state;	// Update ENC state

		lastticks = ticks;	// Update ticks

	} // rotary button has been pressed - exit the loop
	
	return;
}

/**************************** INTERRUPT HANDLERS ******************************/

/**
 * @brief FIT_Handler is used for Software Pulse Width Detection. It both counts the HIgh and low times, as well as blinks Decimal Point 1 periodically.
 * 
 */
void FIT_Handler(void)
{	
	static int fit_count = 0;

	// Read value
	gpio_in = (XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL) & channel_mask) >> shftamnt; 	// Read the GPIO for RGB PWM data.

	if((gpio_in_reg == 0) && (gpio_in == 0)){	// Rising Edge
		lcount++;
	}
	else if((gpio_in_reg == 0) && (gpio_in == 1)){	// Negative Edge
		low_count = lcount;
		lcount = 0;
	}
	else if((gpio_in_reg == 1) && (gpio_in == 0)){	// Negative Edge
		high_count = hcount;
		hcount = 0;
	}
	else if((gpio_in_reg == 1) && (gpio_in == 1)){	// Negative Edge
		hcount++;
	}	
	

	if(gpio_in == 1) hcount++;
	if(gpio_in == 0) lcount++;

	

	// Register value
	gpio_in_reg = gpio_in;

	if(fit_count == 1.5*DP_BLINK_PERIOD){
		if((DP_1_MASK ^ NX4IO_SSEG_getSSEG_DATA(SSEGLO))>>25) 
			NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, 1);
		else			
			NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, 0);
	}

}

/**
 * @brief blink_dp1 is responsible for blinking Decimal Point 1 at about once per second.
 * 
 * @param dp_cnt The active count which determines when to toggle the Decimal Point.
 */
void blink_dp1(unsigned int *dp_cnt){
	// Decimal Point blink routine
		if(*dp_cnt == DP_BLINK_PERIOD){
			*dp_cnt = 0;
			if((DP_1_MASK ^ NX4IO_SSEG_getSSEG_DATA(SSEGLO))>>25) NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, 1);
				else			NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, 0);
		}
		*dp_cnt = *dp_cnt + 1;
}

/**
 * @brief This routine initializes the SSEG display at the start of the program.
 * 
 */
void initial_sseg_state(void){
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT7, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT6, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT5, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT4, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT3, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT2, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT0, false);
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT1, 0x1E);
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, 0x1E);
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT3, 0x1E);
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT2, 0x1E);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, 0x0);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT2, 0x0);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, 0x0);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT4, 0x0);
}

/**
 * @brief button_scan is called when a button press is detected to determine which button was pressed and alter the associated value (val, sat) based on which putton pressed.
 * 
 * @param val val is the "V" of HSV. Altered by up and down buttons.
 * @param sat sat is the "S" of HSV. Altered by left and right buttons.
 * @param hue hue is the "H" of HSV. Altered by
 * 
 */
void button_scan(unsigned char *val, unsigned char *sat, unsigned char *hue){
		// check BTNU and clear count if it's pressed
		// update the count if it is not
		if (NX4IO_isPressed(BTNU))
		{
			if(*val <= 255) *val = (*val << 1) | 0x1;
			else *val = *val;
			*val = *val & 0xFF;
			while(NX4IO_isPressed(BTNU));
		}
		if (NX4IO_isPressed(BTND))
		{
			if(*val >= 1) *val = (*val>> 1) & 0x7F;
			else *val = *val;
			*val = *val & 0xFF;
			while(NX4IO_isPressed(BTND));

		}
		if (NX4IO_isPressed(BTNL))
		{
			if(*sat >=1) *sat = (*sat >> 1)  & 0x7F;
			else *sat = *sat;
			*sat = *sat & 0xFF;
			while(NX4IO_isPressed(BTNL));
		}
		if (NX4IO_isPressed(BTNR))
		{
			if(*sat <= 255) *sat = (*sat << 1) | 0x1;
			else *sat = *sat;
			*sat = *sat & 0xFF;
			while(NX4IO_isPressed(BTNR));
		}

		// Guard the tick value.
		if(*val > 255) *val = 255;
		if(*sat > 255) *sat = 255;

}

/**
 * @brief update_oled handles the writes to the OLEDrgb Pmod display. 
 * 
 * @param oled_rgb_val The passed value is the 565 format rgb value, but re-arranged in order to match the OLED display color to the RGB LED.
 * @param hue The hue value written to the OLED display.
 * @param sat The sat value written to the OLED display.
 * @param val The val value written to the OLED display.
 * @param rgb_val The non-rearranged 565 rgb value format used to match color between the rectangle, font, and RGB led.
 */
void update_oled(u16 *oled_rgb_val, unsigned char *hue, unsigned char *sat, unsigned char *val, u16 *rgb_val){
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst, *oled_rgb_val); // Not correct args names
		// Clear the screen
		for(int i = 0; i<6; i++){
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, i, 0);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"            ");
		}
		
		// Write the screen Values
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"H:        ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"S:        ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 4);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"V:        ");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 2);
		PMDIO_putnum(&pmodOLEDrgb_inst, *hue, 10);
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);
		PMDIO_putnum(&pmodOLEDrgb_inst, *sat, 10);
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 4);
		PMDIO_putnum(&pmodOLEDrgb_inst, *val, 10);
		OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 48, 12, 80, 48, *rgb_val, TRUE, *rgb_val );
}

/**
 * @brief update_sseg writes the digit values produced by the NX4IO duty cycle and Software or Hardware duty cycle detection. 
 * 
 * @param digi1 Value written to DIGIT1 of the SSEGLO register.
 * @param digi2 Value written to DIGIT2 of the SSEGLO register.
 * @param digi3 Value written to DIGIT3 of the SSEGLO register.
 * @param digi4 Value written to DIGIT4 of the SSEGLO register.
 */
void update_sseg(	unsigned char *digi1, 
					unsigned char *digi2, 
					unsigned char *digi3, 
					unsigned char *digi4){

	// Write to SSEG. Ordering not 1:1 in the derived system.
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, *digi4);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT2, *digi1);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, *digi2);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT4, *digi3);

}

/**
 * @brief update_duty simply writes the NX4IO RGB led duty cycle suing the 565 rgb formatted value produced by OLEDrgb_HSVBuilder() function. Ensures all LEDs are always enabled, even if not driven.
 * 
 * @param rgb_val The duty cycle values written to the NX4IO RGBled driver registers. 565 rgb format.
 */
void update_duty(u16 rgb_val){

	NX4IO_RGBLED_setDutyCycle(RGB1, ((rgb_val>>11) & 0x1F), ((rgb_val>>5) & 0x3F), (rgb_val & 0x1F));
	NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
}

/**
 * @brief calculate_dc is more of a multiplexer for the software and hardware pwdetect values than the name indicates. Does calculations for the single digit values to be displayed on the SSEG using the HW (hardpwdet_value) and SW (dc) duty cycle values.
 * 
 * @param NX4IO_duty_cycle The calculated duty cycle based on the 8-bit R, G, and B values calculated from the Hue, Sat, and Val variables. Dependent on switches[3:2] values.
 * @param digi1 The SSEG Digit 1 value to be displayed in the proper NX4IO_SSEG_SetDigit() format.
 * @param digi2 The SSEG Digit 2 value to be displayed in the proper NX4IO_SSEG_SetDigit() format.
 * @param digi3 The SSEG Digit 3 value to be displayed in the proper NX4IO_SSEG_SetDigit() format.
 * @param digi4 The SSEG Digit 4 value to be displayed in the proper NX4IO_SSEG_SetDigit() format.
 * @param pwdet_select The selector based on the Switch[0] value. Determines which Digit 1 and Digit 2 values to display from hardware or software duty cycle calculations.
 * @param hardpwdet_value The value read from the GPIO1 input register. The calculated duty cycle from pwdet.v module in 8 bit format. 
 */
void calculate_dc(	unsigned int *NX4IO_duty_cycle, 
				unsigned char *digi1, 
				unsigned char *digi2,
				unsigned char *digi3,
				unsigned char *digi4,
				unsigned int *pwdet_select,
				volatile u8  *hardpwdet_value
				){


	// Calculated PW.
	if(*NX4IO_duty_cycle > 99){
		*digi2 = 0x9;
		*digi1 = 0x9;
	}
	else{
		*digi2 = (*NX4IO_duty_cycle / 10);
		*digi1 = *NX4IO_duty_cycle % 10;
	}

	// HW-SW switch.
	if(*pwdet_select){

		*hardpwdet_value = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL); 	// Read the GPIO1 port to obtain HWPWDET data.
		
		// Hardware detect
		*digi4 = *hardpwdet_value / 10;
		*digi3 = *hardpwdet_value % 10;

	}
	else{// Software detect
		*digi4 = dc / 10;
		*digi3 = dc % 10;
	} 

	// Quality assurance.
	if(*digi2 > 10) *digi2 = 0x9;
}

/**
 * @brief channel_select is called to provide filter variables for extracting 1 bit signals from the concatenated R, G, and B signals driving the RGBled.
 * 
 * @param switch_value The value of the switch bank.
 * @param NX4IO_duty_cycle The calculated duty cycle based on the 24-bit value "brgbj8" returned by my_hsv_builder(), passed into *rbg_val used in this function. 
 * @param rgb_val The brgb8 value produced by my_hsv_builder which represents the duty cycle for each Red, Blue, and Green 
 */
void channel_select(unsigned int *switch_value, unsigned int *NX4IO_duty_cycle, unsigned int *rgb_val ){
		// Extracts the calculated duty cycle based on the switch[3:2] value.
		// Provides mask and shift values for the RGBled driver signals from gpio_in based on switch[3:2] values.
		switch((*switch_value & SWITCH23_MASK) >> 2){
			case 0: channel_mask = 0x4; shftamnt = 2; *NX4IO_duty_cycle = ((((*rgb_val>>16) & 0xFF)*100)/255); break;	//Red signal
			case 1: channel_mask = 0x1; shftamnt = 0; *NX4IO_duty_cycle = ((((*rgb_val>>8) & 0xFF)*100)/255); break;	// Green Signal
			case 2: channel_mask = 0x2; shftamnt = 1; *NX4IO_duty_cycle = (((*rgb_val & 0xFF)*100)/255); break;	// Blue Signal
			default: break;
		}
}

/**
 * @brief my_hsv_builder is a rip from the PMODOLEDrgb Driver code, repurposed to return 8 bit RGB values.
 * 
 * @param hue The hue value, passed by reference as an 8 bit value.
 * @param sat The saturation value, passed by reference as an 8 bit value. 
 * @param val The value, passed by reference as an 8 bit value.
 * @return int is returned as a 24 bit value consisting of the concatenated Red, Green, and Blue calculated values of Duty Cycle per the hue, sat, and val values.
 */
int my_hsv_builder(unsigned char *hue, unsigned char *sat, unsigned char * val){
	
	u8 region, remain, p, q, t;
   	u8 R, G, B;
	region = *hue / 43;
	remain = (*hue - (region * 43)) * 6;
	p = (*val * (255 - *sat)) >> 8;
	q = (*val * (255 - ((*sat * remain) >> 8))) >> 8;
	t = (*val * (255 - ((*sat * (255 - remain)) >> 8))) >> 8;

	switch (region) {
	case 0:
		R = *val;
		G = t;
		B = p;
		break;
	case 1:
		R = q;
		G = *val;
		B = p;
		break;
	case 2:
		R = p;
		G = *val;
		B = t;
		break;
	case 3:
		R = p;
		G = q;
		B = *val;
		break;
	case 4:
		R = t;
		G = p;
		B = *val;
		break;
	default:
		R = *val;
		G = p;
		B = q;
		break;
	}
	return ( (R<<16) | (G<<8) | B );
}

/**
 * @brief swdetect is called when the Software Pulse Width Detection Duty Cycle is calculated.
 * 
 */
void swdetect(void){

	int scalar = 0;
	int period = 0;

	switch(channel_mask){
			case 0x4:  scalar = 17; break;	//Red signal
			case 0x1:  scalar = 8; break;	// Green Signal
			case 0x2:  scalar = 17; break;	// Blue Signal
			default: scalar = 0; break;
		}

	// Period of the measured signal
	period = high_count + low_count;
	

	// The formula for calculating Duty Cycle. High time divided by the period.
	dc = ((high_count * 100*scalar)/(low_count + high_count));
	if(dc > 99) dc = 99;

}

/**
 * @brief exit_func is the last function to run once the ENC button is pressed, ending the loop operation.
 * 
 */
void exit_func(void){
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT7, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT6, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT5, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT4, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT3, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT2, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT0, false);
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT1, 0x1E);
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, 0x1E);
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT3, 0x1E);
	NX4IO_SSEG_setDigit(SSEGHI, DIGIT2, 0x1E);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, 0x1E);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT2, 0x1E);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, 0x1E);
	NX4IO_SSEG_setDigit(SSEGLO, DIGIT4, 0x1E);

	update_duty(0);	// Update RGBled


}
