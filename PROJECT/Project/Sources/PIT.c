/*! @file
 *
 *  @brief Routines for controlling Periodic Interrupt Timer (PIT) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the periodic interrupt timer (PIT).
 *
 *  @author Corey Stidston & Menka Mehta
 *  @date 2017-04-27
 */
/*!
 **  @addtogroup pit_module PIT module documentation
 **  @{
 */
#include "PIT.h"
#include "MK70F12.h"
#include "OS.h"
#include "LEDs.h"
#include "types.h"

static uint32_t PIT_moduleClk;
static void * PITArguments; //pointer to userArguments funtion
static void (*PITCallback)(void *); //pointer to userCallback function

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk, void (*userFunction)(void*), void* userArguments)
{

  PITSemaphore = OS_SemaphoreCreate(0); //Create PIT Semaphore

  PITArguments = userArguments; //Globally accessible (userArguments)
  PITCallback = userFunction; //Globally accessible (userFunction)
  PIT_moduleClk = moduleClk;

  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;  // Enable clock gate PIT
  //PIT_MCR - PIT Module Control Register | pg1340/2275 K70Manual
  //PIT_MCR &= ~PIT_MCR_MDIS_MASK; //module disable - enabled to allow any kind of setup to PIT
  //PIT_MCR = PIT_MCR_FRZ_MASK; //freeze timer in debug module

  PIT_MCR |= PIT_MCR_MDIS_MASK;

  // Initialise NVICs for PIT | pg 97/2275 k70 manual
  //IRQ 68(seconds interrupt) mod 32 for channel 0 ??? why channel 0?
  NVICICPR2 =  (1 << 4);	 //clears pending  interrupts on PIT channel 0 using IRQ value
  NVICISER2 =  (1 << 4);  //sets/enables interrupts from PIT channel 0

  PIT_MCR &= ~PIT_MCR_MDIS_MASK; //module disable - enabled to allow any kind of setup to PIT

  //Enable interrupts
  //PIT_TCTRL0 - Timer Control Register |TIE - Timer Interrupt Enable |pg 1343/2275 k70 manual
  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK; //enable interrupts for PIT-channel 0


  //PIT_Enable(true);

  return true;
}

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set(const uint32_t period, const bool restart)
{
  //pg 1346/2275 K70 Manual
  uint32_t  freqHz = 1e9 / period;
  uint32_t cycleCount = PIT_moduleClk / freqHz;
  uint32_t triggerVal = cycleCount - 1;

  //TSV - Timer Start Value.
  PIT_LDVAL0 = PIT_LDVAL_TSV(triggerVal); //timer LOad Value registers

  if (restart)
  {
    PIT_Enable(false);
    PIT_Enable(true);
  }
}

/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable(const bool enable)
{
  if (enable == true)
  {
    PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK;
  }
  else
  {
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK;
  }
}

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT_ISR(void)
{
  OS_ISREnter();
  PIT_TFLG0 |= PIT_TFLG_TIF_MASK; //Acknowledge interrupt
  OS_SemaphoreSignal(PITSemaphore); //Signal PIT Semaphore
  OS_ISRExit();

//  if (PITCallback)
//  {
//    (*PITCallback)(PITArguments);
//  }

}










/*!
 ** @}
 */
