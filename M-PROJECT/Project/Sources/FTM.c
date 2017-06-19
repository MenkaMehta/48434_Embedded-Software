/*! @file
 *
 *  @brief Routines for setting up the flexible timer module (FTM) on the TWR-K70F120M.
 *
 *  This contains the functions for operating the flexible timer module (FTM).
 *
 *  @author Corey Stidston and Menka Mehta
 *  @date 2017-04-29
 */
/*!
 **  @addtogroup ftm_module FTM module documentation
 **  @{
 */
#include "FTM.h"
#include "MK70F12.h"
#include "LEDs.h"
#include "OS.h"

#define NO_OF_CHANNELS 8					// Number of channels in flexible timer module
#define FIXED_FREQUENCY_CLOCK 2

static void (*FTMCallback[NO_OF_CHANNELS])(void *); //pointer to userCallback function
static void* FTMArguments[NO_OF_CHANNELS]; //pointer to userArguments function

/*! @brief Sets up the FTM before first use.
 *
 *  Enables the FTM as a free running 16-bit counter.
 *  @return bool - TRUE if the FTM was successfully initialized.
 */
bool FTM_Init()
{
  FTM0Semaphore = OS_SemaphoreCreate(0); //Create FTM0 Semaphore

  SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;  	// pg 356/2275 k70 -Enable clock gate
  //FTM_SCx = Status and Control -contains the overflow status flag and control bits used to configure the interrupt enable
  //lab3 requirements: The clock source for the FTM should be the fixed frequency clock (MCGFFCLK)
  //The fixed frequency clock for each FTM is MCGFFCLK -pg 167 k70


  //FTM0 Register instances (pg1217 - 1223 k70 manual)
  //The CNT register contains the FTM counter value.
  FTM0_CNTIN = ~FTM_CNTIN_INIT_MASK;			// Checks initial value of counter for space
  FTM0_MOD = FTM_MOD_MOD_MASK;				// Initialises FTM counter by writing to CNT
  FTM0_CNT = ~FTM_CNT_COUNT_MASK;			// Checks counter value
  FTM0_SC |= FTM_SC_CLKS(FIXED_FREQUENCY_CLOCK);	// Enable FTM overflow interrupts, up counting mode


  // Initialise NVICs for FTM0 | pg 97/2275 k70 manual
  // IRQ = 62 mod 32 = 30.
  NVICICPR1 = (1<<(62 % 32));                   	// Clears pending interrupts on FMT0 module
  NVICISER1 = (1<<(62 % 32));                   	// Enables interrupts on FTM0 module

  return true; //FTM successfully initialised

}

/*! @brief Sets up a timer channel.
 *
 *  @param aFTMChannel is a structure containing the parameters to be used in setting up the timer channel.
 *    channelNb is the channel number of the FTM to use.
 *    delayCount is the delay count (in module clock periods) for an output compare event.
 *    timerFunction is used to set the timer up as either an input capture or an output compare.
 *    ioType is a union that depends on the setting of the channel as input capture or output compare:
 *      outputAction is the action to take on a successful output compare.
 *      inputDetection is the type of input capture detection.
 *    userFunction is a pointer to a user callback function.
 *    userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the timer was set up successfully.
 *  @note Assumes the FTM has been initialized.
 */
bool FTM_Set(const TFTMChannel* const aFTMChannel)
{
  //refer to structs given in FTM.h
  if (aFTMChannel->timerFunction == TIMER_FUNCTION_INPUT_CAPTURE)
  {
    //Channel (n) Status And Control (pg 1219/2275)
    //The channel mode can be set up as an Input Capture channel
    //For input Capture MSnB:MSnA = 00;
    FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_MSA_MASK; // set 0
    FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_MSB_MASK; // set 0
  }
  else
  {
    //Set controls for output compare MSnB:MSnA = 01;
    FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_MSB_MASK; // set 0
    FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_MSA_MASK; // set 1
  }

  //See K70 Reference Manual on page 1219/2275
  //From the structs in FTM.h file.
  switch (aFTMChannel->ioType.inputDetection)
  {
    case 1:						//Capture on rising edge only (ELSnB:ELSnA = 01)
      FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_ELSB_MASK;
      FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_ELSA_MASK;
      break;
    case 2:						// Capture on falling edge only (ELSnB:ELSnA =10)
      FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_ELSB_MASK;
      FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_ELSA_MASK;
      break;
    case 3:							// Capture on rising or falling edge (ELSnB:ELSnA =11)
      FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_ELSB_MASK;
      FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_ELSA_MASK;
      break;
    default:							// Pin not used for FTM, revert to GPIO  (ELSnB:ELSnA =00)
      FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_ELSB_MASK;
      FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_ELSA_MASK;
      break;
  }

  FTMCallback[aFTMChannel->channelNb] = aFTMChannel->userFunction;
  FTMArguments[aFTMChannel->channelNb] = aFTMChannel->userArguments;
  return true;
}

/*! @brief Starts a timer if set up for output compare.
 *
 *  @param aFTMChannel is a structure containing the parameters to be used in setting up the timer channel.
 *  @return bool - TRUE if the timer was started successfully.
 *  @note Assumes the FTM has been initialized.
 */
bool FTM_StartTimer(const TFTMChannel* const aFTMChannel)
{
  if (aFTMChannel->channelNb < NO_OF_CHANNELS)	//validate channel
  {
    if (aFTMChannel->timerFunction == TIMER_FUNCTION_OUTPUT_COMPARE)
    {
      FTM0_CnV(aFTMChannel->channelNb) = FTM0_CNT + aFTMChannel->delayCount;	// Sets the channels initial countss

      // If any event on the channel has occurred, clear the channel flag
      if (FTM0_CnSC(aFTMChannel->channelNb) & FTM_CnSC_CHF_MASK)
      {
	FTM0_CnSC(aFTMChannel->channelNb) &= ~FTM_CnSC_CHF_MASK;
      }

      //CHIE - channel interrupt enable
      FTM0_CnSC(aFTMChannel->channelNb) |= FTM_CnSC_CHIE_MASK; //enables channel interrupts

      return true; //Timer successfully initialised.
    }
  }
  return false; //Not successful
}

void __attribute__ ((interrupt)) FTM0_ISR(void)
{
  uint8_t channelNb;

  OS_ISREnter();

  for (channelNb = 0; channelNb < NO_OF_CHANNELS; channelNb++)
  {
    // Check if interrupt is enabled for channel and Check if the flag is set for that channel
    if ((FTM0_CnSC(channelNb) & FTM_CnSC_CHIE_MASK) && (FTM0_CnSC(channelNb) & FTM_CnSC_CHF_MASK))
    {
      FTM0_CnSC(channelNb) &= ~FTM_CnSC_CHF_MASK;

      //Disable interrupt
      FTM0_CnSC(channelNb) &= ~FTM_CnSC_CHIE_MASK;

      OS_SemaphoreSignal(FTM0Semaphore); //Signal FTM Semaphore

      //Callback function
//      if (FTMCallback[channelNb])
//      {
//	(*FTMCallback[channelNb])(FTMArguments[channelNb]);
//      }
    }
  }
  OS_ISRExit();
}

/*!
 * @}
 */
