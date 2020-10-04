#include <stdint.h>     // Variable definitions for C99 Standard
#include <stdbool.h>    // Boolean definitions for the C99 Standard

//#define TARGET_IS_BLIZZARD_RB1    // For rom.h, defines TivaC type
#include "inc/tm4c123gh6pm.h"   // def. for the interrupt and register assignments on the Tiva C Series device on the launchPad board
#include "inc/hw_memmap.h"      // Macros defining the memory map
#include "inc/hw_types.h"       // Common macros for TivaC
#include "inc/hw_adc.h"         // Macros for ADC hardware
#include "inc/hw_udma.h"        // Macros for uDMA registers
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
#include "driverlib/udma.h"     // Drivers and macros for uDMA Controller
#include "driverlib/pin_map.h"  // Mapping of peripherals and pins

// Library error routine
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
}
#endif

// uDMA Types
enum BUFFERSTATUS { EMPTY, FILLING, FULL };

// Pins
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

// Buffers
#define ADC_SAMPLE_BUF_SIZE     64
#define MEM_BUFFER_SIZE         1024
char cBuff[5];                                  // UART Input buffer
static uint32_t g_ui32SrcBuf[MEM_BUFFER_SIZE];
static uint32_t g_ui32DstBuf[MEM_BUFFER_SIZE];
static uint16_t ADC_Out1[ADC_SAMPLE_BUF_SIZE];
static uint16_t ADC_Out2[ADC_SAMPLE_BUF_SIZE];
static enum BUFFERSTATUS BufferStatus[2];

// uDMA control table aligned to 1024-byte boundary
#pragma DATA_ALIGN(ucControlTable, 1024)
uint8_t ucControlTable[1024];

// Variables
// Define transfer counter
static uint32_t g_ui32MemXferCount = 0;
// Define errors counters
static uint32_t g_ui32DMAErrCount = 0u;
static uint32_t g_ui32BadISR = 0u;
// Accumulator for system ticks
static uint32_t g_ui32SysTickCount;

// Prototypes
// Custom Functions
void UARTprintLedStatus(uint32_t, uint8_t);
// Initialize Functions
void ConfigureUART(void);
void init_DMA();
void init_GPIO();
void init_TIMER();
void init_ADC();
// Interrupt Handlers
void ADCseq0Handler(void);
void uDMAErrorHandler(void);
void GPIOF0IntHandler(void);
void SysTickIntHandler(void);
void Timer0AIntHandler(void);

// Program Entry
int main (void) {
    // Variables
    uint32_t i, average1, average2, samples_taken;
    volatile uint32_t ui32TempAvg;
    volatile uint32_t ui32TempValueC;
    volatile uint32_t ui32TempValueF;
    // System clock to 160Mhz (PLL= 400Mhz / 2.5 = 160Mhz)
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // Enable Peripherals
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlDelay(30u);

    // Enable System Clock
    MAP_SysTickPeriodSet( SysCtlClockGet() / 100000u );
    //SysTickIntRegister( SysTickIntHandler );
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    // Initialize
    BufferStatus[0] = FILLING;
    BufferStatus[1] = EMPTY;
    samples_taken = 0u;

    // Configure Peripherals
    ConfigureUART();
    UARTprintf("Terminal Active\n");

    init_DMA();
    init_GPIO();
    init_ADC();
    init_TIMER();

    MAP_IntMasterEnable(); // Enable Global Interrupts
    MAP_TimerEnable(TIMER0_BASE, TIMER_A); // Start
    while(1) {
        // Read into Buffer1 or Buffer2 if the other is full
        if (BufferStatus[0u] == FULL) {
             average1 = 0u; // Reset average to accumulate
             // Sum and clear values from buffer
             for(i = 0u; i < ADC_SAMPLE_BUF_SIZE; i++) {
                 average1 += ADC_Out1[i];
                 ADC_Out1[i] = 0u;
             }
             BufferStatus[0u] = EMPTY; // Clear buffer
             // Mark and enable to be transfered into ADC_Out1
             uDMAChannelTransferSet( UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
                                    (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out1, ADC_SAMPLE_BUF_SIZE );
             uDMAChannelEnable( UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT );
             // Update sample count and average
             samples_taken += ADC_SAMPLE_BUF_SIZE;
             average1 = (average1 + ( ADC_SAMPLE_BUF_SIZE / 2u)) / ADC_SAMPLE_BUF_SIZE;
         }
         if (BufferStatus[1u] == FULL) {
             average2 = 0u; // Reset average to accumulate
             // Sum and clear values from buffer
             for(i = 0u; i < ADC_SAMPLE_BUF_SIZE; i++) {
                 average2 += ADC_Out2[i];
                 ADC_Out2[i] = 0u;
         }
             BufferStatus[1u] = EMPTY;  // Clear buffer
             // Mark and enable to be transfered into ADC_Out2
             uDMAChannelTransferSet( UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
                                    (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out2, ADC_SAMPLE_BUF_SIZE );
             uDMAChannelEnable( UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT );
             // Update sample count and average
             samples_taken += ADC_SAMPLE_BUF_SIZE;
             average2 = (average2 + ( ADC_SAMPLE_BUF_SIZE / 2u)) / ADC_SAMPLE_BUF_SIZE;
            // UARTprintf("\t%d\t\t%d\t\t%d\r", ui32TempValueC, ui32TempValueF, samples_taken);
         }
       // Calculate Temperature in C and F
       ui32TempAvg = (average1 + average2) / 2;
       ui32TempValueC = (1475 - ((2475 * ui32TempAvg)) / 4096)/10;
       ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;

       // Read in from UART Buffer
       UARTgets(cBuff, 2);
       // Controls and UART Menu Display
       if (cBuff[0] == 'R') { // Red LEDs
           UARTprintf("Red LED ON\n");
           GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
       } else if( cBuff[0] == 'r') {
           UARTprintf("Red LED OFF\n");
           GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
       }

       if (cBuff[0] == 'G') { // Green LEDs
           UARTprintf("Green LED ON\n");
           GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, GREEN_LED);
       } else if( cBuff[0] == 'g') {
           UARTprintf("Green LED OFF\n");
           GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0);
       }

       if (cBuff[0] == 'B') { // Blue LEDs
           UARTprintf("Blue LED ON\n");
           GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED);
       } else if( cBuff[0] == 'b') {
           UARTprintf("Blue LED OFF\n");
           GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
       }

       if (cBuff[0] == 'T') // Temperature
           UARTprintf("Temperature C: %d\n", ui32TempValueC);
       else if( cBuff[0] == 't')
           UARTprintf("Temperature F: %d\n", ui32TempValueF);

       if (cBuff[0] == 'S') {   // LED Staus
           UARTprintf("LED Status:\n");
           SysCtlDelay(1000);

           UARTprintf("Red LED:\t");
           UARTprintLedStatus(GPIO_PORTF_BASE, RED_LED);
           UARTprintf("Green LED:\t");
           UARTprintLedStatus(GPIO_PORTF_BASE, GREEN_LED);
           UARTprintf("Blue LED:\t");
           UARTprintLedStatus(GPIO_PORTF_BASE, BLUE_LED);
       }
    }
}

void UARTprintLedStatus(uint32_t ui32Port, uint8_t ui8Pins) {
    // Display ON or OFF to UART
    volatile int32_t status = GPIOPinRead(ui32Port, ui8Pins);
    SysCtlDelay(1000);
    if (status == 0)
       UARTprintf("\tOFF\n");
    else
       UARTprintf("\tON\n");
}

void ConfigureUART(void) {
    // GPIO setup for UART

    // Enable UART GPIO pins
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Configure GPIO pins for UART mode
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Use internal 16MHz as clock
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Initialize for I/O console
    UARTStdioConfig(0, 115200, 16000000);
}

void init_DMA() {
    // Initialize uDMA on ADC CH0 in Ping-Pong Mode
    uDMAEnable();

    uDMAControlBaseSet(ucControlTable);

    uDMAChannelAttributeDisable( UDMA_CHANNEL_ADC0, UDMA_ATTR_ALTSELECT |
                                 UDMA_ATTR_HIGH_PRIORITY| UDMA_ATTR_REQMASK);

    uDMAChannelAttributeEnable( UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST );

    uDMAChannelControlSet( UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                           UDMA_SIZE_16 |UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    uDMAChannelControlSet( UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
                           UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);


    uDMAChannelTransferSet( UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG,
                           (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out1, ADC_SAMPLE_BUF_SIZE);
    uDMAChannelTransferSet( UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,
                           (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out2, ADC_SAMPLE_BUF_SIZE);

    uDMAChannelEnable(UDMA_CHANNEL_ADC0);
}

void init_GPIO() {
    // Initialize GPIO pins
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock GPIO Port F
    GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
    // Set LED pins as outputs
    GPIOPinTypeGPIOOutput( GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED );
    // Set PF0 as input
    GPIOPinTypeGPIOInput( GPIO_PORTF_BASE, GPIO_PIN_0);

    //register the interrupt handler for PF0
    GPIOIntRegister(GPIO_PORTF_BASE, GPIOF0IntHandler);
     //SW2 goes low when pressed
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    //enable interrupts on PF0
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
}

void init_TIMER() {
    // Initialize Timer 0
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/16000 - 1);
    // Enable ADC trigger output
    MAP_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    // Timer stops when in debug mode
    MAP_TimerControlStall(TIMER0_BASE, TIMER_A, true);
    // Enable interrupt on Timer 0 A
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void init_ADC() {
    // ADC CH0 Timer Triggered for uDMA
    // Set GPIO PIN to read in ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    SysCtlDelay(80);

    // Set ADC clock as PIOSC / 1 at half rate (16MHz/1) / 2 = 8 MHz
    ADCClockConfigSet( ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1 );
    // Wait for configuration to finish
    SysCtlDelay(10);

    // Disable interrupts on ADC0
    IntDisable( INT_ADC0SS0 );
    // Disable interrupts on sequencer 0
    ADCIntDisable( ADC0_BASE, 0 );
    // Disable sample sequencer 0
    ADCSequenceDisable( ADC0_BASE, 0 );

    // Set interrupt function
    //ADCIntRegister( ADC0_BASE , 0, ADCseq0Handler );
    // Using ADC0, sequencer 1, processor triggered, highest priority
    ADCSequenceConfigure( ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0 );
    // Configuration reading from internal temperature sensor as last in sequence.
    // Causes interrupt when complete
    ADCSequenceStepConfigure( ADC0_BASE, 0, 0, ADC_CTL_TS | ADC_CTL_END | ADC_CTL_IE );
    // Re-enable sequencer 0

    // Re-enable sequencer 0
    ADCSequenceEnable(ADC0_BASE, 0 );
    // Clear status flag before writing to ADC
    ADCIntClear( ADC0_BASE, 0 );
    // Enable DMA for sequencer 0
    ADCSequenceDMAEnable( ADC0_BASE, 0 );
    // Re-enable interrupts for sequencer 0
    ADCIntEnable( ADC0_BASE, 0 );
    // Re-enable interrupts on ADC 0
    IntEnable( INT_ADC0SS0 );
}

void ADCseq0Handler (void){
   // ADCSequenceDataGet(ADC0_BASE, 0, ui32ADC0Value);
//    UARTprintf("Seq Interrupt");
    ADCIntClear(ADC0_BASE, 0);

    if((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT) == UDMA_MODE_STOP)
        && (BufferStatus[0] == FILLING))
    {
        BufferStatus[0] = FULL;
        BufferStatus[1] = FILLING;
    } else if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT) == UDMA_MODE_STOP)
            && (BufferStatus[1] == FILLING))
    {
        BufferStatus[0] = FILLING;
        BufferStatus[1] = FULL;
    }
}

// uDMA error handler
void uDMAErrorHandler(void) {
    uint32_t ui32Status;

    ui32Status = uDMAErrorStatusGet();

    if(ui32Status)
    {
        uDMAErrorStatusClear();
        g_ui32DMAErrCount++;
    }
}

void GPIOF0IntHandler(void) {
    // Clear interrupt flag on pin F0
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    // Toggle all LEDs
    int32_t status = GPIOPinRead(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);
    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, 0xFFFFFFFF ^ status);
}

void SysTickIntHandler(void) {
    g_ui32SysTickCount++;
//    UARTprintf("Sys Tick ");
}

// For debugging purposes
void Timer0AIntHandler(void) {
    // Clear the interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
//    UARTprintf("Timer Int ");
}
