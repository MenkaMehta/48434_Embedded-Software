/* ###################################################################
 **     Filename    : main.c
 **     Project     : Lab6
 **     Processor   : MK70FN1M0VMJ12
 **     Version     : Driver 01.01
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
 **     Abstract    :
 **         Main module.
 **         This module contains user's application code.
 **     Settings    :
 **     Contents    :
 **         No public methods
 **
 ** ###################################################################*/
/*!
 ** @file main.c
 ** @version 6.0
 ** @brief
 **         Main module.
 **         This module contains user's application code.
 */
/*!
 **  @addtogroup main_module main module documentation
 **  @{
 */
/* MODULE main */

// CPU module - contains low level hardware initialization routines
#include "Cpu.h"

// Simple OS
#include "OS.h"

// Analog functions
#include "analog.h"
#include "UART.h"
#include "types.h"
#include "packet.h"
#include "LEDs.h"
#include "Flash.h"
#include "PIT.h"
#include <string.h>
// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 3

/****************************************PRIVATE FUNCTION DECLARATION**************************************/
static void InitThread(void* data);
static void PacketThread(void* data);
static void PITThread(void* data);

//static OS_ECB *InitSemaphore;

// Thread stacks
/****************************************THREAD STACKS*****************************************************/
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t InitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t ReceiveThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t TransmitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PacketThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PITThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

/****************************************GLOBAL VARS*******************************************************/
const static uint32_t BAUD_RATE = 115200;
const static uint32_t MODULE_CLOCK = CPU_BUS_CLK_HZ;

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {1, 2, 3};

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
} TAnalogThreadData;


//ADC DAC +-10V 16bit

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1
  },
  {
     .semaphore = NULL,
     .channelNb = 2
  }

};


void PITCallback(void *arg)
{
  LEDs_Toggle(LED_GREEN);
}

//void LPTMRInit(const uint16_t count)
//{
//  // Enable clock gate to LPTMR module
//  SIM_SCGC5 |= SIM_SCGC5_LPTIMER_MASK;
//
//  // Disable the LPTMR while we set up
//  // This also clears the CSR[TCF] bit which indicates a pending interrupt
//  LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;
//
//  // Enable LPTMR interrupts
//  LPTMR0_CSR |= LPTMR_CSR_TIE_MASK;
//  // Reset the LPTMR free running counter whenever the 'counter' equals 'compare'
//  LPTMR0_CSR &= ~LPTMR_CSR_TFC_MASK;
//  // Set the LPTMR as a timer rather than a counter
//  LPTMR0_CSR &= ~LPTMR_CSR_TMS_MASK;
//
//  // Bypass the prescaler
//  LPTMR0_PSR |= LPTMR_PSR_PBYP_MASK;
//  // Select the prescaler clock source
//  LPTMR0_PSR = (LPTMR0_PSR & ~LPTMR_PSR_PCS(0x3)) | LPTMR_PSR_PCS(1);
//
//  // Set compare value
//  LPTMR0_CMR = LPTMR_CMR_COMPARE(count);
//
//  // Initialize NVIC
//  // see p. 91 of K70P256M150SF3RM.pdf
//  // Vector 0x65=101, IRQ=85
//  // NVIC non-IPR=2 IPR=21
//  // Clear any pending interrupts on LPTMR
//  NVICICPR2 = NVIC_ICPR_CLRPEND(1 << 21);
//  // Enable interrupts from LPTMR module
//  NVICISER2 = NVIC_ISER_SETENA(1 << 21);
//
//  //Turn on LPTMR and start counting
//  LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;
//}
//
//void __attribute__ ((interrupt)) LPTimer_ISR(void)
//{
//  // Clear interrupt flag
//  LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;
//
//  // Signal the analog channels to take a sample
//  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
//    (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
//}


/*! @brief Initialises modules.
 *
 */
static void InitModulesThread(void* pData)
{
  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Generate the global analog semaphores
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    AnalogThreadData[analogNb].semaphore = OS_SemaphoreCreate(0);

  // Initialise the low power timer to tick every 10 ms
 // LPTMRInit(10);


   bool packetStatus = Packet_Init(BAUD_RATE, MODULE_CLOCK);
   bool flashStatus  = Flash_Init();
   bool ledStatus = LEDs_Init();
   PIT_Init(MODULE_CLOCK, &PITCallback, (void *)0);
   bool PITStatus = PIT_Init(MODULE_CLOCK, &PITCallback, (void *)0);
   PIT_Set(10000000, true);



  if (packetStatus && ledStatus && PITStatus) //&& RTCStatus && FTMStatus && AccelStatus && flashStatus
  {
    LEDs_On(LED_ORANGE);  //Tower was initialized correctly
  }
  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*!
 * @brief Runs packet thread
 */
void PacketThread(void* data)
{
  Packet_Put(TOWER_STARTUP_COMM, TOWER_STARTUP_PAR1, TOWER_STARTUP_PAR2, TOWER_STARTUP_PAR3);
  Packet_Put(TOWER_NUMBER_COMM, TOWER_NUMBER_PAR1, TowerNumber->s.Lo, TowerNumber->s.Hi);
  Packet_Put(TOWER_VERSION_COMM, TOWER_VERSION_V, TOWER_VERSION_MAJ, TOWER_VERSION_MIN);
  Packet_Put(TOWER_MODE_COMM, TOWER_MODE_PAR1, TowerMode->s.Lo, TowerMode->s.Hi);

  for (;;)
  {
    if (Packet_Get()) //Check if there is a packet in the retrieved data
    {
      LEDs_On(LED_BLUE);
      //FTM_StartTimer(&packetTimer);
      Packet_Handle();
    }
  }
}

/*!
 * @brief Runs pit thread
 */
void PITThread(void* data)
{
  for (;;)
  {
    //Wait on PIT Semaphire
    OS_SemaphoreWait(PITSemaphore, 0);
    // Signal the analog channels to take a sample
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);

    LEDs_Toggle(LED_GREEN);
  }
}

/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void AnalogLoopbackThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphore, 0);
    // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);
    // Put analog sample
    Analog_Put(analogData->channelNb, analogInputValue);
  }
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialise low-level clocks etc using Processor Expert code


  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread, NULL, &InitModulesThreadStack[THREAD_STACK_SIZE - 1],0); // Highest priority
  error = OS_ThreadCreate(ReceiveThread, NULL, &ReceiveThreadStack[THREAD_STACK_SIZE-1], 4);
  error = OS_ThreadCreate(TransmitThread, NULL, &TransmitThreadStack[THREAD_STACK_SIZE-1], 5);
  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE-1], 7);
  //already made - error = OS_ThreadCreate(PITThread, NULL, &PITThreadStack[THREAD_STACK_SIZE-1], 3);

  // Create threads for 2 analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalogLoopbackThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  }

  error = OS_ThreadCreate(PITThread, NULL, &PITThreadStack[THREAD_STACK_SIZE-1], 6);
  // Start multithreading - never returns!
  OS_Start();
}

/*!
 ** @}
 */
