///* ###################################################################
// **     Filename    : main.c
// **     Project     : Lab2
// **     Processor   : MK70FN1M0VMJ12
// **     Version     : Driver 01.01
// **     Compiler    : GNU C Compiler
// **     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
// **     Abstract    :
// **         Main module.
// **         This module contains user's application code.
// **     Settings    :
// **     Contents    :
// **         No public methods
// **
// ** ###################################################################*/
///*!
// ** @file main.c
// ** @version 2.0
// ** @brief
// **         Main module.
// **         This module contains user's application code.
// */
///*!
// **  @addtogroup main_module main module documentation
// **  @{
// */
///* MODULE main */
//
//
//// CPU module - contains low level hardware initialization routines
//#include "Cpu.h"
//#include "Events.h"
//#include "PE_Types.h"
//#include "PE_Error.h"
//#include "PE_Const.h"
//#include "IO_Map.h"
//#include "Flash.h"
//#include "LEDs.h"
//#include "packet.h"
//#include "types.h"
//#include "UART.h"
////#include "RTC.h"
//#include "PIT.h"
////#include "FTM.h"
////#include "median.h"
////#include "I2C.h"
////#include "accel.h"
//#include <string.h>
//#include "OS.h"
//
//#define THREAD_STACK_SIZE 100
//
///****************************************PRIVATE FUNCTION DECLARATION**************************************/
//static void FTM0Callback(void *arg);
//static void RTCCallback(void *arg);
//static void PITCallback(void *arg);
//static void SlidingWindow(uint8_t* const array, const size_t arraylength, const uint8_t newValue);
//static void HandleMedianData();
//static void InitThread(void* data);
//static void PacketThread(void* data);
//static void PITThread(void* data);
//static void RTCThread(void* data);
//static void AccelThread(void* data);
//static void I2CThread(void *data);
//
///****************************************THREAD STACKS*****************************************************/
//static uint32_t InitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t PacketThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t ReceiveThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t TransmitThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t PITThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t RTCThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t FTM0ThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t AccelThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//static uint32_t I2CThreadStack[THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
//
//static OS_ECB *InitSemaphore;
//
///****************************************GLOBAL VARS*******************************************************/
//const static uint32_t BAUD_RATE = 115200;
//const static uint32_t MODULE_CLOCK = CPU_BUS_CLK_HZ;
//
///*!
// * @brief Contains the latest accelerometer data
// */
//static uint8_t AccReadData[3] = {0};
///*!
// * @brief The latest bytes of accelerometer data which were sent.
// */
//static uint8_t AccelSendHistory[3] = {0};
///*!
// * @brief The latest bytes of X read from the accelerometer .
// */
//static uint8_t AccXHistory[3] = {0};
///*!
// * @brief The latest bytes of Yread from the accelerometer .
// */
//static uint8_t AccYHistory[3] = {0};
///*!
// * @brief The latest bytes of Z read from the accelerometer .
// */
//static uint8_t AccZHistory[3] = {0};
//
//static uint8_t AccTimerRunningFlag = 0;
//
////TFTMChannel configuration for FTM timer
//TFTMChannel packetTimer = {
//  0, 															//channel
//  CPU_MCGFF_CLK_HZ_CONFIG_0,			//delay count
//  TIMER_FUNCTION_OUTPUT_COMPARE,	//timer function
//  TIMER_OUTPUT_HIGH,							//ioType
//  FTM0Callback,										//User function
//  (void*) 0												//User arguments
//};
//
//const static TAccelSetup ACCEL_SETUP = {
//  .moduleClk = CPU_BUS_CLK_HZ,
//  .dataReadyCallbackFunction = &HandleMedianData,
//  .dataReadyCallbackArguments = 0,
//  .readCompleteCallbackFunction = &HandleMedianData,
//  .readCompleteCallbackArguments = 0,
//};
//
///****************************************PRIVATE FUNCTION DEFINITION***************************************/
//
///*!
// * @brief slidingwindow Shifts the elements of an array one to the left.
// * @param array The array to shifting window.
// * @param arraylength The length of the array.
// * @param newValue The new value to insert at index 0.
// */
//void SlidingWindow(uint8_t* const array, const size_t arraylength, const uint8_t newValue)
//{
//  for (size_t i = (arraylength -1); i > 0; i--)
//  {
//    array[i] = array [i-1];
//  }
//  array[0] = newValue;
//}
//
///*!
// * @brief Run on the main thread to handle new accelerometer data.
// */
//void HandleMedianData()
//{
//  if (Accel_GetMode() == ACCEL_INT)
//  {
//    Accel_ReadXYZ(AccReadData);
//    Packet_Put(0x10, AccReadData[0], AccReadData[1], AccReadData[2]);
//    return;
//  }
//
//  //shifting history
//  SlidingWindow(AccXHistory,3,AccReadData[0]);
//  SlidingWindow(AccYHistory,3,AccReadData[1]);
//  SlidingWindow(AccZHistory,3,AccReadData[2]);
//
//  uint8_t xMedian = Median_Filter3(AccXHistory[0],AccXHistory[1],AccXHistory[2] );
//  uint8_t yMedian = Median_Filter3(AccYHistory[0],AccYHistory[1],AccYHistory[2] );
//  uint8_t zMedian = Median_Filter3(AccZHistory[0],AccZHistory[1],AccZHistory[2] );
//
//  if ((xMedian!= AccelSendHistory[0]) | (yMedian != AccelSendHistory[1]) | (zMedian != AccelSendHistory[2]))
//  {
//    AccelSendHistory[0] = xMedian;
//    AccelSendHistory[1] = yMedian;
//    AccelSendHistory[2] = zMedian;
//
//    Packet_Put(0x10, xMedian, yMedian, zMedian);
//  }
//}
//
///*!
// * @brief Initialise the initial functions
// */
//void TowerInit(void)
//{
//  bool packetStatus = Packet_Init(BAUD_RATE, MODULE_CLOCK);
//  bool flashStatus  = Flash_Init();
//  bool ledStatus = LEDs_Init();
//  bool PITStatus = PIT_Init(MODULE_CLOCK, &PITCallback, (void *)0);
//  PIT_Set(500e6, true);
//  bool RTCStatus = RTC_Init(&RTCCallback, (void *)0);
//
//  bool FTMStatus = FTM_Init();
//  FTM_Set(&packetTimer);
//
//  bool AccelStatus = Accel_Init(&ACCEL_SETUP);
//
//  if (packetStatus && flashStatus && ledStatus && PITStatus && RTCStatus && FTMStatus && AccelStatus)
//  {
//    LEDs_On(LED_ORANGE);	//Tower was initialized correctly
//  }
//}
//
///*!
// * @brief Runs tower init and delete thread
// */
//void InitThread(void* data)
//{
//  OS_ERROR error;
//
//  for (;;)
//  {
//    // Wait on Init Semaphore
//    OS_SemaphoreWait(InitSemaphore, 0);
//    TowerInit();	//Initialize tower peripheral modules
//    error = OS_ThreadDelete(0);
//  }
//}
//
///*!
// * @brief Runs packet thread
// */
//void PacketThread(void* data)
//{
//  Packet_Put(TOWER_STARTUP_COMM, TOWER_STARTUP_PAR1, TOWER_STARTUP_PAR2, TOWER_STARTUP_PAR3);
//  Packet_Put(TOWER_NUMBER_COMM, TOWER_NUMBER_PAR1, TowerNumber->s.Lo, TowerNumber->s.Hi);
//  Packet_Put(TOWER_VERSION_COMM, TOWER_VERSION_V, TOWER_VERSION_MAJ, TOWER_VERSION_MIN);
//  Packet_Put(TOWER_MODE_COMM, TOWER_MODE_PAR1, TowerMode->s.Lo, TowerMode->s.Hi);
//
//  for (;;)
//  {
//    if (Packet_Get())	//Check if there is a packet in the retrieved data
//    {
//      LEDs_On(LED_BLUE);
//      FTM_StartTimer(&packetTimer);
//      Packet_Handle();
//    }
//  }
//}
//
///*!
// * @brief Runs pit thread
// */
//void PITThread(void* data)
//{
//  for (;;)
//  {
//    //Wait on PIT Semaphire
//    OS_SemaphoreWait(PITSemaphore, 0);
//
//    LEDs_Toggle(LED_GREEN);
//    //The code stops working with the following code.
//    if (Accel_GetMode() == ACCEL_POLL)
//    {
//      //TODO: ASYNC I2C
////      Accel_ReadXYZ(AccReadData);
////      //HandleMedianData();
////      Packet_Put(0x10, AccReadData[0], AccReadData[1], AccReadData[2]);
//    }
//  }
//}
//
///*!
// * @brief Runs accel thread
// */
//void AccelThread(void* data)
//{
//  for (;;)
//  {
//    //Wait on Accel Semaphore
//    OS_SemaphoreWait(AccelSemaphore, 0);
//    Accel_ReadXYZ(AccReadData);
//    //HandleMedianData();
//    LEDs_Toggle(LED_GREEN);
//    Packet_Put(0x10, AccReadData[0], AccReadData[1], AccReadData[2]);
//    (void)OS_SemaphoreSignal(AccelSemaphore);
//  }
//}
//
///*!
// * @brief Runs RTC thread
// */
//void RTCThread(void* data)
//{
//  for (;;)
//  {
//    //Wait on RTC Semaphore
//    OS_SemaphoreWait(RTCSemaphore, 0);
//
//    uint8_t h, m ,s;
//    RTC_Get(&h, &m, &s); //Get hours, mins, secs
//    Packet_Put(0x0c, h, m, s); //Send to PC
//    LEDs_Toggle(LED_YELLOW); //Toggle Yellow LED
//  }
//}
//
///*!
// * @brief Runs FTM0 thread
// */
//void FTM0Thread(void *data)
//{
//  for (;;)
//  {
//    //Wait on the FTM0 Semaphore
//    OS_SemaphoreWait(FTM0Semaphore, 0);
//    LEDs_Off(LED_BLUE);
//  }
//}
//
///*!
// * @brief Runs I2C thread
// */
//void I2CThread(void *data)
//{
//  for (;;)
//  {
//    //Wait on the I2C Semaphore
//    OS_SemaphoreWait(I2CSemaphore, 0);
//    HandleMedianData();
//  }
//}
//
///*lint -save  -e970 Disable MISRA rule (6.3) checking. */
//int main(void)
///*lint -restore Enable MISRA rule (6.3) checking. */
//{
//  /* Write your local variable definition here */
//
//  OS_ERROR error;
//  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
//  PE_low_level_init();
//  /*** End of Processor Expert internal initialization.                    ***/
//
//  /* Write your code here */
//
//
//  OS_Init(CPU_CORE_CLK_HZ, false);
//
//  // Create Initialisation Semaphore
//  InitSemaphore = OS_SemaphoreCreate(1);
//
//  // Create threads
//  error = OS_ThreadCreate(InitThread, NULL, &InitThreadStack[THREAD_STACK_SIZE-1], 0);
//  error = OS_ThreadCreate(ReceiveThread, NULL, &ReceiveThreadStack[THREAD_STACK_SIZE-1], 1);
//  error = OS_ThreadCreate(TransmitThread, NULL, &TransmitThreadStack[THREAD_STACK_SIZE-1], 2);
//  error = OS_ThreadCreate(PITThread, NULL, &PITThreadStack[THREAD_STACK_SIZE-1], 3);
//  error = OS_ThreadCreate(RTCThread, NULL, &RTCThreadStack[THREAD_STACK_SIZE-1], 4);
//  error = OS_ThreadCreate(FTM0Thread, NULL, &FTM0ThreadStack[THREAD_STACK_SIZE-1], 5);
//  error = OS_ThreadCreate(PacketThread, NULL, &PacketThreadStack[THREAD_STACK_SIZE-1], 6);
//  error = OS_ThreadCreate(AccelThread, NULL, &AccelThreadStack[THREAD_STACK_SIZE-1], 7);
//  error = OS_ThreadCreate(I2CThread, NULL, &I2CThreadStack[THREAD_STACK_SIZE-1], 8);
//
//  // Start threads
//  OS_Start();
//
//  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
//  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
//  #ifdef PEX_RTOS_START
//    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
//  #endif
//  /*** End of RTOS startup code.  ***/
//  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
//  for(;;){}
//  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
//} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/
//
////RTCCallback function from RTC_ISR
//void RTCCallback(void *arg)
//{
//  uint8_t h, m ,s;
//  RTC_Get(&h, &m, &s);			//Get hours, mins, secs
//  Packet_Put(0x0c, h, m, s);//Send to PC
//  LEDs_Toggle(LED_YELLOW);	//Toggle Yellow LED
//}
//
////PITCallback function from PIT_ISR
//void PITCallback(void *arg)
//{
//  LEDs_Toggle(LED_GREEN);
//  if (Accel_GetMode() == ACCEL_POLL)
//  {
//    Accel_ReadXYZ(AccReadData);
//    //HandleMedianData();
//    Packet_Put(0x10, AccReadData[0], AccReadData[1], AccReadData[2]);
//  }
//}
//
////FTM0Callback function from FTM_ISR
//void FTM0Callback(void *arg)
//{
//  LEDs_Off(LED_BLUE);
//}
//
//
///* END main */
///*!
// ** @}
// */
///*
// ** ###################################################################
// **
// **     This file was created by Processor Expert 10.5 [05.21]
// **     for the Freescale Kinetis series of microcontrollers.
// **
// ** ###################################################################
// */
