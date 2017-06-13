/*! @file
 *
 * @brief implementations of functions which control the LEDs.
 *
 * @author Corey Stidston & Menka Mehta
 * @date 2017-04-18
 *
 */
/*!
 * @addtogroup led_module LEDs module documentation
 * @{
 */
/****************************************HEADER FILES****************************************************/
#include "LEDs.h"
#include "MK70F12.h"

/****************************************PUBLIC FUNCTION DEFINITION***************************************/
/*! @brief Sets up the LEDs before first use.
 *
 *  @return bool - TRUE if the LEDs were successfully initialized.
 */
bool LEDs_Init(void)
{
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

  GPIOA_PSOR |= (LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE);

  PORTA_PCR10 &= ~PORT_PCR_MUX_MASK;
  PORTA_PCR11 &= ~PORT_PCR_MUX_MASK;
  PORTA_PCR28 &= ~PORT_PCR_MUX_MASK;
  PORTA_PCR29 &= ~PORT_PCR_MUX_MASK;

  PORTA_PCR10 |= PORT_PCR_MUX(1);
  PORTA_PCR11 |= PORT_PCR_MUX(1);
  PORTA_PCR28 |= PORT_PCR_MUX(1);
  PORTA_PCR29 |= PORT_PCR_MUX(1);

  //Set as a General Purpose Output
  GPIOA_PDDR |= (LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE);
  return true;
}

/*! @brief Turns an LED on.
 *
 *  @param color The color of the LED to turn on.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_On(const TLED color)
{
  GPIOA_PCOR = color;
}

/*! @brief Turns off an LED.
 *
 *  @param color The color of the LED to turn off.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Off(const TLED color)
{
  GPIOA_PSOR = color;
}

/*! @brief Toggles an LED.
 *
 *  @param color THe color of the LED to toggle.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Toggle(const TLED color)
{
  GPIOA_PTOR = color;
}
/*!
 * @}
 */
