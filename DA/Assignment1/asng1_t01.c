#include <stdint.h>     // Variable definitions for C99 Standard
#include <stdbool.h>    // Boolean definitions for the C99 Standard

//#define TARGET_IS_BLIZZARD_RB1    // For rom.h, defines TivaC type
#include "inc/tm4c123gh6pm.h"   // def. for the interrupt and register assignments on the Tiva C Series device on the launchPad board
#include "inc/hw_memmap.h"      // Macros defining the memory map
#include "inc/hw_types.h"       // Common macros for TivaC
//#include "inc/hw_ints.h"      // Macros for TivaC interrupts
//#include "driverlib/rom.h"    // Macros for calling functions in ROM
#include "driverlib/rom_map.h"  // Macros for default calling functions in ROM otherwise in Flash
#include "utils/uartstdio.h"    // UART Console Driver Functions
#include "driverlib/interrupt.h"// Macros for NVIC Controller API of DriverLib
#include "driverlib/gpio.h"     // API GPIO ports
#include "driverlib/debug.h"    // Macros for debugging driverlib
#include "driverlib/sysctl.h"   // Driver for SysTick
#include "driverlib/systick.h"  // Driver for SysTick timer in NVIC
#include "driverlib/adc.h"      // Drivers for ADC
#include "driverlib/timer.h"    // Drivers for timers
#include "driverlib/uart.h"     // Macros for UART
#include "driverlib/pin_map.h"  // Mapping of peripherals and pins

// Library error routine
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
}
#endif

// Pins
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

// Variables
uint32_t ui32Period;
uint32_t ui32ADC0Value[4]; // Stores ADC value (size of ADC sequencer)
volatile uint32_t ui32TempAvg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;

// Prototypes
// Initialize Functions
void init_ADC();
void init_GPIO();
void init_TIMER();
void init_UART();
// Interrupt Handlers
void Timer0AIntHandler(void);
void GPIOF0IntHandler(void);

// Program Entry
int main (void) {

    //System clock to 40Mhz (PLL= 400Mhz / 10 = 40Mhz)
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // Enable Peripherals
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // Running at default rate of 1Msps
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Configure Peripherals
    init_ADC();
    init_GPIO();
    init_TIMER();
    init_UART();
    UARTprintf("Terminal Active\n");

    IntMasterEnable();                  // Enable Master Interrupt
    TimerEnable(TIMER0_BASE, TIMER_A);  // Enable Timer 0
    ADCSequenceEnable(ADC0_BASE, 1);    // Enable Sequencer 1
    while(1) {
    }
}

void init_ADC() {
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); // Using ADC0, sequencer 1, processor triggered, highest priority
    // All 4 sequencers sampling internal temperature sensor
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS);
    // Samples temperature, sets interrupt flag when done, tell ADC last conversion on sequencer 1
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);
}

void init_GPIO() {
    // Configure the GPIO port for the LED operation.
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);

    //Unlock Pin F0 to use an interrupt on SW2
    SYSCTL_RCGC2_R |= 0x00000020;     // activate clock for Port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock GPIO Port F
    GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
    // only PF0 needs to be unlocked, other bits can't be locked
    GPIO_PORTF_AMSEL_R = 0x00;        // disable analog on PF
    GPIO_PORTF_PCTL_R = 0x00000000;   // PCTL GPIO on PF4-0
    GPIO_PORTF_DIR_R = 0x0E;          // PF4,PF0 in, PF3-1 out
    GPIO_PORTF_AFSEL_R = 0x00;        // disable alt funct on PF7-0
    GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
    GPIO_PORTF_DEN_R = 0x1F;          // enable digital I/O on PF4-0

    //register the interrupt handler for PF0
    GPIOIntRegister(GPIO_PORTF_BASE, GPIOF0IntHandler);
     //SW2 goes low when pressed
    MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    //enable interrupts on PF0
    MAP_GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
}

void init_TIMER() {
    /*
      Configure the timer as periodic, by omission it's in count down mode.
      It counts from the load value to 0 and then resets back to the load value.
    */
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ui32Period = SysCtlClockGet() / 2;  // Period of 0.5s 2Hz
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    MAP_IntEnable(INT_TIMER0A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void init_UART() {
   // GPIO Setup

   // Enable UART GPIO pins
   MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
   MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
   // Configure GPIO pins for UART mode
   MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
   MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
   MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
   // Use internal 16MHz as clock
   UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
   // Initialize for I/O console
   UARTStdioConfig(0, 115200, 16000000);
}

void Timer0AIntHandler(void) {
    // Clear the interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Clear status flag before writing to ADC
    ADCIntClear(ADC0_BASE, 1);
    // Set ADC to trigger with software
    ADCProcessorTrigger(ADC0_BASE, 1);

    // Copy from ADC FIFO to buffer
    ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
    // Calculate average temperature
    ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
    // Convert to C (datasheet's equation)
    ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
    ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;

    UARTprintf("F:%d\t", ui32TempValueF);
    UARTprintf("C:%d\t\n", ui32TempValueC);
}

void GPIOF0IntHandler(void) { //interrupt handler for GPIO pin F0
    // Clear interrupt flag on pin F0
    MAP_GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    // Toggle all LEDs
    int32_t status = GPIOPinRead(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, 0xFFFFFFFF ^ status);
}
