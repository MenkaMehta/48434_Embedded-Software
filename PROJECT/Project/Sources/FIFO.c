/*! @file
 *
 *  @brief FIFO buffer function implementations
 *
 *  @author Corey Stidston and Menka Mehta
 *  @date 2017-04-18
 */
/*!
 * @addtogroup fifo_module FIFO module documentation
 * @{
 */

/****************************************HEADER FILES****************************************************/
#include "FIFO.h"
#include "PE_Types.h"
#include "Cpu.h"
#include "OS.h"
/****************************************PUBLIC FUNCTION DEFINITION***************************************/

/*! @brief Initialize the FIFO before first use.
 *
 *  @param FIFO A pointer to the FIFO that needs initializing.
 *  @return void
 */
void FIFO_Init(TFIFO * const FIFO)
{
  FIFO->Start = 0;
  FIFO->End = 0;
  FIFO->NbBytes = 0;
  FIFO->BufferAccess = OS_SemaphoreCreate(1); //Create Semaphore (1 to indicate it is not being used)
  FIFO->SpaceAvailable = OS_SemaphoreCreate(FIFO_SIZE); //Create Semaphore which maintains space available in FIFO
  FIFO->ItemsAvailable = OS_SemaphoreCreate(0); //Create Semaphore to wait on space available
}

/*! @brief Put one character into the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct where data is to be stored.
 *  @param data A byte of data to store in the FIFO buffer.
 *  @return bool - TRUE if data is successfully stored in the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
void FIFO_Put(TFIFO * const FIFO, const uint8_t data)
{
  OS_SemaphoreWait(FIFO->SpaceAvailable, 0); // Wait on space available
  OS_SemaphoreWait(FIFO->BufferAccess, 0); // Get exclusive access


  FIFO->Buffer[FIFO->End] = data; 	//Put data into FIFO buffer
  FIFO->NbBytes++; 			//Number of bytes in FIFO increases
  FIFO->End++; 			//Next available position iterates
  if (FIFO->End == FIFO_SIZE-1) FIFO->End = 0; //Check whether the FIFO is full, reset

  OS_SemaphoreSignal(FIFO->BufferAccess); // Signal exclusive access
  OS_SemaphoreSignal(FIFO->ItemsAvailable); // Signal space available
}

/*! @brief Get one character from the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct with data to be retrieved.
 *  @param dataPtr A pointer to a memory location to place the retrieved byte.
 *  @return bool - TRUE if data is successfully retrieved from the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
void FIFO_Get(TFIFO * const FIFO, uint8_t * const dataPtr)
{
  OS_SemaphoreWait(FIFO->ItemsAvailable, 0); //Wait on items available
  OS_SemaphoreWait(FIFO->BufferAccess, 0); //Wait on exclusive access

  *dataPtr = FIFO->Buffer[FIFO->Start]; //Data = Array[Start]
  FIFO->Start++; //Moves to the next element in the array
  FIFO->NbBytes--;
  if (FIFO->Start == FIFO_SIZE-1) FIFO->Start = 0;

  OS_SemaphoreSignal(FIFO->BufferAccess); //Signal no exclusive access taking place
  OS_SemaphoreSignal(FIFO->SpaceAvailable); //Signal space available in FIFO
}


/*!
 * @}
 */
