//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2012-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 1.1 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"



#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "inc/hw_gpio.h"
#include "driverlib/pin_map.h"


// Defines for drv8842 pins
// MO_PWM6->PC4, MO_PWM7->PC5
#define NRESET_PORT GPIO_PORTF_BASE
#define NRESET_PIN GPIO_PIN_0
#define NSLEEP_PORT "WRONG!"
#define NSLEEP "WRONG!"
#define NFAULT_PORT GPIO_PORTB_BASE
#define NFAULT_PIN GPIO_PIN_7
#define DECAY_PORT GPIO_PORTB_BASE
#define DECAY_PIN GPIO_PIN_6
#define IN2_PORT GPIO_PORTA_BASE
#define IN2_PIN GPIO_PIN_4
#define IN1_PORT GPIO_PORTA_BASE
#define IN1_PIN GPIO_PIN_3
#define I0_PORT GPIO_PORTA_BASE
#define I0_PIN GPIO_PIN_2
#define I1_PORT GPIO_PORTF_BASE
#define I1_PIN GPIO_PIN_4
#define I2_PORT GPIO_PORTD_BASE
#define I2_PIN GPIO_PIN_7
#define I3_PORT GPIO_PORTD_BASE
#define I3_PIN GPIO_PIN_6
#define I4_PORT GPIO_PORTC_BASE
#define I4_PIN GPIO_PIN_7

#define IN1_PWM_PORT GPIO_PORTC_BASE
#define IN1_PWM_PIN GPIO_PIN_4
#define IN1_PWM PWM_OUT_6
#define IN2_PWM_PORT GPIO_PORTC_BASE
#define IN2_PWM_PIN GPIO_PIN_5
#define IN2_PWM PWM_OUT_7

#define NRESET_SUBS_PORT GPIO_PORTC_BASE
#define NRESET_SUBS_PIN GPIO_PIN_6

//pe_1:táp
//pe_0:motor
#define SUPPLY_PWR_PORT GPIO_PORTE_BASE
#define SUPPLY_PWR_PIN GPIO_PIN_1
#define MOTOR_CURR_PORT GPIO_PORTE_BASE
#define MOTOR_CURR_PIN GPIO_PIN_0
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint16_t ui16Adjust1 = 5;
volatile uint16_t ui16Adjust2 = 512;
volatile uint16_t PWM_FREQ = 40000;
void
ConfigureSSI0(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinConfigure(GPIO_PD3_SSI3TX);
    GPIOPinConfigure(GPIO_PD2_SSI3RX);
    GPIOPinConfigure(GPIO_PD1_SSI3FSS);
    GPIOPinConfigure(GPIO_PD0_SSI3CLK);
    GPIOPinTypeSSI(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 );
    SSIClockSourceSet(SSI3_BASE, SSI_CLOCK_PIOSC);
    SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1,
    SSI_MODE_MASTER, 20000, 16);
    SSIEnable(SSI3_BASE);
}
void
ConfigurePWM_GPIO(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOInput(NFAULT_PORT,NFAULT_PIN);
	GPIOPinTypeGPIOInput(NRESET_PORT,NRESET_PIN);
	GPIOPinTypeGPIOInput(IN2_PORT,IN2_PIN);
	GPIOPinTypeGPIOInput(IN1_PORT,IN1_PIN);// PWM FROM PC4/5 will be routed here....

	GPIOPinTypeGPIOOutput(NRESET_SUBS_PORT,NRESET_SUBS_PIN);

	GPIOPinTypeGPIOOutput(DECAY_PORT,DECAY_PIN);
	GPIOPinTypeGPIOOutput(DECAY_PORT,DECAY_PIN);
	GPIOPinTypeGPIOOutput(I0_PORT,I0_PIN);
	GPIOPinTypeGPIOOutput(I1_PORT,I1_PIN);
	GPIOPinTypeGPIOOutput(I2_PORT,I2_PIN);
	GPIOPinTypeGPIOOutput(I3_PORT,I3_PIN);
	GPIOPinTypeGPIOOutput(I4_PORT,I4_PIN);

	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	GPIOPinTypePWM(IN1_PWM_PORT,IN1_PWM_PIN);
	GPIOPinTypePWM(IN2_PWM_PORT,IN2_PWM_PIN);
	ROM_GPIOPinConfigure(GPIO_PC4_M0PWM6);
	ROM_GPIOPinConfigure(GPIO_PC5_M0PWM7);

	PWMGenConfigure(PWM0_BASE,PWM_GEN_3,PWM_GEN_MODE_DOWN);// PWM_GEN_3??? REALLY? why not 6/7? Tuti?
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, (SysCtlClockGet()/PWM_FREQ) -1);

	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, ui16Adjust1 * ( (SysCtlClockGet()/PWM_FREQ) -1) / 1000);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ui16Adjust2 * ( (SysCtlClockGet()/PWM_FREQ) -1) / 1000);
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
	//PWMOutputInvert(PWM0_BASE,PWM_OUT_6_BIT | PWM_OUT_7_BIT,true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);

	}
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);


    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}
void  DRV8842_OUTPUTS()
{
	GPIOPinWrite(NRESET_SUBS_PORT,NRESET_SUBS_PIN,NRESET_SUBS_PIN);
	GPIOPinWrite(NFAULT_PORT,NFAULT_PIN,NFAULT_PIN);
	GPIOPinWrite(DECAY_PORT,DECAY_PIN,DECAY_PIN);
	GPIOPinWrite(I0_PORT,I0_PIN,I0_PIN);
	GPIOPinWrite(I1_PORT,I1_PIN,I1_PIN);
	GPIOPinWrite(I2_PORT,I2_PIN,I2_PIN);
	GPIOPinWrite(I3_PORT,I3_PIN,I3_PIN);
	GPIOPinWrite(I4_PORT,I4_PIN,I4_PIN);


}
void DisablePWMOnZero()
{
    if((ui16Adjust1 * ( (SysCtlClockGet()/PWM_FREQ)-1)) / 1000 == 0)
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false);
    else
    	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    if((ui16Adjust2 * ( (SysCtlClockGet()/PWM_FREQ)-1)) / 1000 == 0)
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
    else
    	PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
	}
void ConfigureADC()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH3);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH3);
	ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_CH2|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);
   // ROM_GPIOPinConfigure(GPIO_PE0_ADC3);
    //ROM_GPIOPinConfigure(GPIO_PA1_U0TX);

}

//*****************************************************************************
//
// Print "Hello World!" to the UART on the evaluation board.
//
//*****************************************************************************
uint32_t ui32ADC0Value[4];
uint32_t addr = 0xFFFF;
uint32_t * getdata;
uint32_t getdata2;
int
main(void)
{
	float ADCVals[4];
	int i = 0;
	int dir = 0;
    //volatile uint32_t ui32Loop;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Initialize the UART.
    //
    ConfigureUART();
    ConfigureSSI0();
    ConfigurePWM_GPIO();
    DisablePWMOnZero();
    DRV8842_OUTPUTS();
    ConfigureADC();
    //
    // Hello!
    //
    ui16Adjust1 = 0;
    ui16Adjust2 = 0;
    UARTprintf("Hello, world!\n");
    UARTprintf("%d\n",SysCtlClockGet());
    UARTprintf("Full PWM CYCLE CLOCKS:%d\n",((SysCtlClockGet()/PWM_FREQ) -1));
    UARTprintf("PC4:%d\n",ui16Adjust1 * ( (SysCtlClockGet()/PWM_FREQ)-1) / 1000);
    UARTprintf("PC5:%d\n",ui16Adjust2 * ( (SysCtlClockGet()/PWM_FREQ)-1) / 1000);
   // while(1);
    while(1)
    {
        //
        // Turn on the BLUE LED.
        //
       /* GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for a bit.
        //
        SysCtlDelay(SysCtlClockGet() / 10 / 3);

        //
        // Turn off the BLUE LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);*/

        //
        // Delay for a bit.
        //
        SysCtlDelay(SysCtlClockGet() / 100);
        SSIDataPutNonBlocking(SSI3_BASE, addr);
      // // while(SSIBusy(SSI3_BASE));
        SSIDataGetNonBlocking(SSI3_BASE,&getdata2);
        UARTprintf("%u    ",(getdata2 ) &0x3FFF);
        //UARTprintf("dir: %u, ui16Adjust2: %u\n",dir,ui16Adjust2 );

        //***adc
        ADCIntClear(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC0_BASE, 1);
        while(!ADCIntStatus(ADC0_BASE, 1, false))
        {
        }
        ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);

        ADCVals[0] = ui32ADC0Value[0]*0.00069113276*1000;
        ADCVals[1] = ui32ADC0Value[1]*0.00975986975*1000;
        ADCVals[2] = ui32ADC0Value[2]*0.00069113276*1000;
        ADCVals[3] = ui32ADC0Value[3]*0.00975986975*1000;
        UARTprintf("MOTOR0: %u mA, MOTOR1: %u A, POWER0: %u V, POWER1: %u \n "
        		,(uint32_t)ADCVals[0],(uint32_t)ADCVals[2],
        		(uint32_t)ADCVals[1],(uint32_t)ADCVals[3]);
        //****adc 3.3/(4095*5*0.2) = 0.0008058608, (1.8 + 20 )/(1.8*4095) = 0.00295753629

        DisablePWMOnZero();

    	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, (ui16Adjust1 * ( (SysCtlClockGet()/PWM_FREQ)-1)) / 1000);
    	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, (ui16Adjust2 * ( (SysCtlClockGet()/PWM_FREQ)-1)) / 1000);
    	if (ui16Adjust2 +3 >=1000 ) dir =1;
    	if (  ui16Adjust2 -3 <= 0) dir =0;
    	if (!dir) ui16Adjust2 += 3;
    	if (dir) ui16Adjust2 -= 3;
        //UARTprintf("\n");

    }
}
