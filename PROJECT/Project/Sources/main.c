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
#include "Events.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
// Analog functions
#include "analog.h"
#include "UART.h"
#include "types.h"
#include "packet.h"
#include "LEDs.h"
#include "FTM.h"
#include "Flash.h"
#include "PIT.h"
#include <string.h>
//new
#include <stdio.h>
#include <math.h>

// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 3
//new
#define MAX_FREQUENCY 52.5
#define MIN_FREQUENCY 47.5
#define ANALOG_WINDOW_SIZE 16
#define KINVERSE 0.14
#define KVINVERSE 13.5
#define KEINVERSE 80
#define ALPHAINVERSE 0.02
#define ALPHAVINVERSE 1
#define ALPHAEINVERSE 2
//new
//uint8_t PITcount = 0;

/****************************************PRIVATE FUNCTION DECLARATION**************************************/
static void InitThread(void* data);
static void PacketThread(void* data);
static void PIT0Thread(void* data);
static void PIT1Thread(void* data);
static void FTM0Callback(void *arg);

//static OS_ECB *InitSemaphore;

// Thread stacks
/****************************************THREAD STACKS*****************************************************/
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t InitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t ReceiveThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t TransmitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PacketThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PIT0ThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PIT1ThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t FTM0ThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

/****************************************GLOBAL VARS*******************************************************/
const static uint32_t BAUD_RATE = 115200;
const static uint32_t MODULE_CLOCK = CPU_BUS_CLK_HZ;
#define ANALOG_WINDOW_SIZE 16
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
  int16_t values[ANALOG_WINDOW_SIZE];
  double irms;
  double triptime;
} TAnalogThreadData;


//ADC DAC +-10V 16bit

//TFTMChannel configuration for FTM timer
TFTMChannel packetTimer = {
  0,                              //channel
  CPU_MCGFF_CLK_HZ_CONFIG_0,      //delay count
  TIMER_FUNCTION_OUTPUT_COMPARE,  //timer function
  TIMER_OUTPUT_HIGH,              //ioType
  FTM0Callback,                   //User function
  (void*) 0                       //User arguments
};

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0,
    .values[0] = 0,
    .irms      = 0.0,
    .triptime  = 0.0
  },
  {
    .semaphore = NULL,
    .channelNb = 1,
    .values[0] = 0,
    .irms      = 0.0,
    .triptime  = 0.0
  },
  {
     .semaphore = NULL,
     .channelNb = 2,
     .values[0] = 0,
     .irms      = 0.0,
     .triptime  = 0.0
  }

};

double calculate_Time(double irms)
{
    double time = 0.0;
    switch (characteristic)
    {
      //characteristic mode from the TowerPc
     case INVERSE:
       time = KINVERSE/(pow(irms, ALPHAINVERSE)); //inverse
     case VERYINVERSE:
       time = KVINVERSE/(pow(irms, ALPHAVINVERSE)); //very inverse
     case EXTREMELYINVERSE:
       time = KEINVERSE/(pow(irms, ALPHAEINVERSE)); //extreme inverse
     default:
       time = KINVERSE/(pow(irms, ALPHAINVERSE)); //inverse
    }
    return time;
}

/*
 * calculate RMS
 * http://www.edaboard.com/thread12502.html
 */
double calculate_RMS(int16_t values[])
{
//  uint8_t freq, time;
//  if (freq > MIN_FREQUENCY && freq < MAX_FREQUENCY )
//  {
//    //covert freq into time in ms into us
//    time = (1 / freq) * 1000000;
//    //if freq is 50Hz buffer size 20000/16samples = 1250us

    uint32_t rawrms;
    double rms, irms;

    for (uint8_t count = 0 ; count < 16 ; count++ )
    {
//   rawrms += adcbuf[count] * adcbuf[count];
      rawrms += (*values) * (*values);
      values++;
    }

    rms = (double)rawrms / 16;
    rms = sqrt(rms);

   //the ratio of Irms=1 then Vrms=350mV
    irms = rms/0.35; //Irms should be an array storing the Irms values
    return (irms);

//  }
}


//FTM0Callback function from FTM_ISR
void FTM0Callback(void *arg)
{
  LEDs_Off(LED_BLUE);
}


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
   PIT_Init(MODULE_CLOCK);
   bool PITStatus = PIT_Init(MODULE_CLOCK);
//   PIT_Set(10000000, true);
   PIT0_Set(1250000, true);
   bool FTMStatus = FTM_Init();
   FTM_Set(&packetTimer);


  if (packetStatus && ledStatus && PITStatus && FTMStatus && flashStatus) //&& RTCStatus  && AccelStatus && flashStatus
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
      FTM_StartTimer(&packetTimer);
      Packet_Handle();
    }
  }
}

/*!
 * @brief Runs pit0 thread
 */
void PIT0Thread(void* data)
{
  for (;;)
  {
    //Wait on PIT Semaphire
    OS_SemaphoreWait(PIT0Semaphore, 0);
    // Signal the analog channels to take a sample
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);

    LEDs_Toggle(LED_GREEN);
  }
}

/*!
 * @brief Runs pit1 thread
 */
void PIT1Thread(void* data)
{
  for (;;)
  {
    //Wait on PIT Semaphore
    OS_SemaphoreWait(PIT1Semaphore, 0);
    // Signal the analog channels to take a sample
//    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
//      (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphore);
    PIT1_Enable(false); //disable the pit

    LEDs_Toggle(LED_YELLOW);
  }
}

/*
 * @brief Runs FTM0 thread
 */
void FTM0Thread(void *data)
{
  for (;;)
  {
    //Wait on the FTM0 Semaphore
    OS_SemaphoreWait(FTM0Semaphore, 0);
    LEDs_Off(LED_BLUE);
  }
}

/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void AnalogLoopbackThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)
  static uint8_t count = 0;


  for (;;)
  {

    int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphore, 0);

    //get 16 sample values - voltages
      // Get analog sample
    Analog_Get(analogData->channelNb, &analogInputValue);
    if (count < 16)
    {
      analogData->values[count] = analogInputValue;
     count++;
    }
     else
     {
       count = 0;
       analogData->values[count] = analogInputValue; //get 16 values
       analogData->irms = calculate_RMS(analogData->values); //calculate irms
     }

    if(analogData->irms>=3375.001)// irms>=1.03 16bits(32767 to -32768) ((32767*103)/1000)=3375.001
    {
    //work on tripping if irms < 1.03 its never going to trip
    //if irms >= 1.03 its going to trip after that time.
//    Analog_Put(0,value); //5v times 3000scale
    analogData->triptime = calculate_Time(analogData->irms); //calculate time depending on the irms
    PIT1_Set(analogData->triptime, false); //PIT1_Set(const uint32_t period, const bool restart)
    PIT1_Enable(true); //PIT1_Enable(const bool enable)

    //Put analog sample
    Analog_Put(analogData->channelNb, analogInputValue);
    }
//    else
//    {
//      //irms < 3375.001 then it never trips because time is infinity
//      //Analog_Put(0,0); (0,0) channel0 and value- 0v
//    }
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
  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE-1], 8);
  error = OS_ThreadCreate(FTM0Thread, NULL, &FTM0ThreadStack[THREAD_STACK_SIZE-1], 9);
  //already made - error = OS_ThreadCreate(PITThread, NULL, &PITThreadStack[THREAD_STACK_SIZE-1], 3);

  // Create threads for 3 analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
//    error = OS_ThreadCreate(AnalogLoopbackThread,
//                            &AnalogThreadData[threadNb],
//                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
//                            ANALOG_THREAD_PRIORITIES[threadNb]);
      error = OS_ThreadCreate(AnalogLoopbackThread, &AnalogThreadData[0], &AnalogThreadStacks[0][THREAD_STACK_SIZE-1], 1);
  }


  error = OS_ThreadCreate(PIT0Thread, NULL, &PIT0ThreadStack[THREAD_STACK_SIZE-1], 6);
  error = OS_ThreadCreate(PIT1Thread, NULL, &PIT1ThreadStack[THREAD_STACK_SIZE-1], 7);
  // Start multithreading - never returns!
  OS_Start();
}


/*!
 ** @}
 */



