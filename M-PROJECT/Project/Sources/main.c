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
//new libraries
#include <stdio.h>
#include <math.h>

// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 150
#define NB_ANALOG_CHANNELS 3
//new
//#define MAX_FREQUENCY 52.5
//#define MIN_FREQUENCY 47.5
#define ANALOG_WINDOW_SIZE 16
#define KINVERSE 0.14
#define KVINVERSE 13.5
#define KEINVERSE 80
#define ALPHAINVERSE 0.02
#define ALPHAVINVERSE 1
#define ALPHAEINVERSE 2
#define ANALOG_WINDOW_SIZE 16
#define FIVE_VOLTS 16384 //APPROX 5V

/****************************************PRIVATE FUNCTION DECLARATION**************************************/
static void InitThread(void* data);
static void PacketThread(void* data);
static void PIT0Thread(void* data);
static void FTM0Callback(void *arg);


// Thread stacks
/****************************************THREAD STACKS*****************************************************/
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t InitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t ReceiveThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t TransmitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PacketThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t PIT0ThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t FTMThreadStack[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

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
  int16_t values[ANALOG_WINDOW_SIZE];
  double irms;
  double triptime;
  double newtriptime;
} TAnalogThreadData;

//ADC DAC +-10V 16bit

//TFTMChannel configuration for FTM timer
TFTMChannel packetTimer[NB_ANALOG_CHANNELS] = {
    {
        0,                              //channel
        CPU_MCGFF_CLK_HZ_CONFIG_0,      //delay count
        TIMER_FUNCTION_OUTPUT_COMPARE,  //timer function
        TIMER_OUTPUT_HIGH,              //ioType
        FTM0Callback,                   //User function
        (void*) 0                       //User arguments
    },
    {
        1,                              //channel
        CPU_MCGFF_CLK_HZ_CONFIG_0,      //delay count
        TIMER_FUNCTION_OUTPUT_COMPARE,  //timer function
        TIMER_OUTPUT_HIGH,              //ioType
        FTM0Callback,                   //User function
        (void*) 0                       //User arguments
    },
    {
        2,                              //channel
        CPU_MCGFF_CLK_HZ_CONFIG_0,      //delay count
        TIMER_FUNCTION_OUTPUT_COMPARE,  //timer function
        TIMER_OUTPUT_HIGH,              //ioType
        FTM0Callback,                   //User function
        (void*) 0                       //User arguments
    }

};
//
//    TFTMChannel packetTimer = {
//
//        0,                              //channel
//        CPU_MCGFF_CLK_HZ_CONFIG_0,      //delay count
//        TIMER_FUNCTION_OUTPUT_COMPARE,  //timer function
//        TIMER_OUTPUT_HIGH,              //ioType
//        FTM0Callback,                   //User function
//        (void*) 0                       //User arguments
//
//    };


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
            .triptime  = 0.0,
            .newtriptime = 0.0
        },
        {
            .semaphore = NULL,
            .channelNb = 1,
            .values[0] = 0,
            .irms      = 0.0,
            .triptime  = 0.0,
            .newtriptime = 0.0
        },
        {
            .semaphore = NULL,
            .channelNb = 2,
            .values[0] = 0,
            .irms      = 0.0,
            .triptime  = 0.0,
            .newtriptime = 0.0
        }

    };

double Calculate_Time(double irms)
{
  double time = 0.0;
  switch (characteristic)
  {
    //characteristic mode from the TowerPc
    case INVERSE:
      time = KINVERSE/(pow(irms, ALPHAINVERSE) - 1); //inverse
      break;
    case VERYINVERSE:
      time = KVINVERSE/(pow(irms, ALPHAVINVERSE) - 1); //very inverse
      break;
    case EXTREMELYINVERSE:
      time = KEINVERSE/(pow(irms, ALPHAEINVERSE) - 1); //extreme inverse
      break;
    default:
      time = KINVERSE/(pow(irms, ALPHAINVERSE) - 1); //inverse
      break;
  }
  return time;
}


/*
 * calculate RMS
 * http://www.edaboard.com/thread12502.html
 */
double Calculate_RMS(int16_t values[])
{
  double rawrms;
  double rms, irms;

  for (uint8_t count = 0 ; count < 16 ; count++ )
  {
    double value = *values *  0.00030518509;
    rawrms += (value * value);
    values++;
  }

  rms = (double)rawrms / 16;
  rms = sqrt(rms);

  //the ratio of Irms=1 then Vrms=350mV
  irms = rms/0.35;
  return (irms);

}


//FTM0Callback function from FTM_ISR
void FTM0Callback(void *arg)
{
  Analog_Put(1,FIVE_VOLTS); //5/0.00030518509 covert the value back to voltage - 5V channel1-trip
  LEDs_On(LED_BLUE);
  //  LEDs_Off(LED_BLUE);
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
  PIT0_Set(1250000, true);
  bool FTMStatus = FTM_Init();
  for(int i=0; i< NB_ANALOG_CHANNELS; i++)
  {
    FTM_Set(&packetTimer[i]);
  }
  //  FTM_Set(&packetTimer);
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
  //new
  Packet_Put(CHAR_COMM, CHAR_PAR1, characteristic , 0X0);

  for (;;)
  {
    if (Packet_Get()) //Check if there is a packet in the retrieved data
    {
      //      LEDs_On(LED_BLUE);
      //      FTM_StartTimer(&packetTimer);
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


/*
 * @brief Runs FTM0 thread
 */
void FTMThread(void *data)
{
  OS_ECB* semaphore = *((OS_ECB**)data);
  for (;;)
  {
    //Wait on the FTM0 Semaphore
    //    OS_SemaphoreWait(FTMSemaphore[0], 0);
    //    OS_SemaphoreWait(FTMSemaphore[1], 0);
    //    OS_SemaphoreWait(FTMSemaphore[2], 0);
    OS_SemaphoreWait(semaphore, 0);

    Analog_Put(1,FIVE_VOLTS); //5/0.00030518509 covert the value back to voltage - 5V channel1-trip
    LEDs_On(LED_BLUE);

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

  //  TFTMChannel* timer = &packetTimer[analogData->channelNb];

  for (;;)
  {

    int16_t analogInputValue;
    double percentRemaining;

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
      analogData->irms = Calculate_RMS(analogData->values); //calculate irms
      analogData->triptime = Calculate_Time(analogData->irms);
      Analog_Put(0,0); //initially channel0 -before timing
      Analog_Put(1,0); //initially channel1 -before triping

      if (analogData->irms >= 1.03)
      {
        Analog_Put(0,FIVE_VOLTS); //for the timing channel0
        //timer->delayCount = ((CPU_MCGFF_CLK_HZ_CONFIG_0 * analogData->triptime)/128); ///2
        packetTimer[NB_ANALOG_CHANNELS].delayCount = ((CPU_MCGFF_CLK_HZ_CONFIG_0 * analogData->triptime)/128/2);
        FTM_StartTimer(&packetTimer[NB_ANALOG_CHANNELS]);
        //inverse timing
        analogData->newtriptime = Calculate_Time(analogData->irms);
        if(analogData->newtriptime == analogData->triptime )
        {
          FTM_StartTimer(&packetTimer[NB_ANALOG_CHANNELS]);
        }
        else
        {
          percentRemaining = FTM_PercentageRemaining(&packetTimer[NB_ANALOG_CHANNELS]);
          analogData->newtriptime = Calculate_Time(analogData->irms) * percentRemaining;
          //          packetTimer.delayCount = ((CPU_MCGFF_CLK_HZ_CONFIG_0 * analogData->newtriptime)/128);
          packetTimer[NB_ANALOG_CHANNELS].delayCount = ((CPU_MCGFF_CLK_HZ_CONFIG_0 * analogData->triptime)/128/2);
          //          FTM_StartTimer(timer);
          //          FTM_StartTimer(&packetTimer);
          FTM_StartTimer(&packetTimer[analogData->channelNb]);
        }
      }
      else
      {
        //irms < 1.03 therefore it will never trip coz time is infinity
        Analog_Put(0,0); //set 0 volts for timing-channel 0
        Analog_Put(1,0); //set 0 volts for trip-channel 1
      }
    }

    //Put analog sample
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
  error = OS_ThreadCreate(ReceiveThread, NULL, &ReceiveThreadStack[THREAD_STACK_SIZE-1], 9);
  error = OS_ThreadCreate(TransmitThread, NULL, &TransmitThreadStack[THREAD_STACK_SIZE-1], 10);
  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE-1], 8);
  //  error = OS_ThreadCreate(FTMThread, NULL, &FTMThreadStack[THREAD_STACK_SIZE-1], 5);
  // Create threads for 3 analog loopback channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(FTMThread, &FTMSemaphore[threadNb], &FTMThreadStack[threadNb][THREAD_STACK_SIZE-1], 5 + threadNb);

    error = OS_ThreadCreate(AnalogLoopbackThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  }


  error = OS_ThreadCreate(PIT0Thread, NULL, &PIT0ThreadStack[THREAD_STACK_SIZE-1], 4);

  // Start multithreading - never returns!
  OS_Start();
}


/*!
 ** @}
 */



