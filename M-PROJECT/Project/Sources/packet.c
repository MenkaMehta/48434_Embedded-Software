/*!
 * @file <packet.c>
 *
 * @brief
 *         packet module.
 *         This module contains the code for managing incoming and outgoing packets
 *
 *@author Corey Stidston & Menka Mehta
 * @date 2017-03-29
 */
/*!
 * @addtogroup packet_module packet documentation
 * @{
 */

/****************************************HEADER FILES****************************************************/
#include "packet.h"
#include "UART.h"
#include "MK70F12.h"
#include "types.h"
#include "LEDs.h"
#include "Flash.h"
//#include "RTC.h"
#include "PE_Types.h"
#include "Cpu.h"
//#include "accel.h"


/****************************************GLOBAL VARS*****************************************************/

TPacket Packet;

uint8_t packet_position = 0;	//Used to mark the position of incoming bytes

const uint8_t PACKET_ACK_MASK = 0x80u; //Used to mask out the Acknowledgment bit

uint16union_t volatile *TowerNumber;
uint16union_t volatile *TowerMode;
//new
uint16union_t volatile *charac;

/****************************************PRIVATE FUNCTION DECLARATION***********************************/

bool PacketTest(void);
bool DataToFlash(void);

/****************************************PRIVATE FUNCTION DEFINITION***************************************/

/*! @brief Handles the stored packet
 *
 *  @return bool - True if the calculated checksum is equal to the packet checksum
 */
bool PacketTest(void)
{
  uint8_t calculated_checksum = Packet_Command ^ Packet_Parameter1 ^ Packet_Parameter2 ^ Packet_Parameter3;
  return (calculated_checksum == Packet_Checksum);
}

/*! @brief send datatoFlash
 *
 *  @return bool - TRUE if the data was saved to flash
 */
bool DataToFlash(void)
{
  bool numberAlloc = Flash_AllocateVar((volatile void **) &TowerNumber, sizeof(uint16union_t));
  bool modeAlloc = Flash_AllocateVar((volatile void **) &TowerMode, sizeof(uint16union_t));
  if(numberAlloc && modeAlloc)
  {
    if(TowerNumber->l == 0xFFFF) //If un-programmed
    {
      Flash_Write16((uint16_t volatile *) TowerNumber, S_ID);
    }
    if(TowerMode->l == 0xFFFF)	//If un-programmed
    {
      Flash_Write16((uint16_t volatile *) TowerMode, 0x1);
    }
    return true;
  }
  return false;
}

/****************************************PUBLIC FUNCTION DEFINITION***************************************/

/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  PacketPutSemaphore = OS_SemaphoreCreate(1); //Create Packet Semaphore

  return (UART_Init(baudRate, moduleClk) && DataToFlash());
}

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void) {
  //EnterCritical();
  uint8_t uartData;

  //Checks whether there is data in the RxFIFO and stores it the address pointed by uartData
  UART_InChar(&uartData);
  switch (packet_position)
  {
    //Command byte
    case 0:
      Packet_Command = uartData;
      packet_position++;
      //ExitCritical();
      return false; //Return false, incomplete packet
      break;

      //Parameter1 byte
    case 1:
      Packet_Parameter1 = uartData;
      packet_position++;
      //ExitCritical();
      return false; //Return false, incomplete packet
      break;

      //Parameter2 byte
    case 2:
      Packet_Parameter2 = uartData;
      packet_position++;
      ExitCritical();
      return false; //Return false, incomplete packet
      break;

      //Parameter3 byte
    case 3:
      Packet_Parameter3 = uartData;
      packet_position++;
      //ExitCritical();
      return false; //Return false, incomplete packet
      break;

      //Checksum byte
    case 4:
      Packet_Checksum = uartData;

      if (PacketTest())
      {
	packet_position = 0;
	//ExitCritical();
	return true; //Return true, complete packet
      }
      //The Checksum doesn't match
      //Shift the packets down
      Packet_Command = Packet_Parameter1;
      Packet_Parameter1 = Packet_Parameter2;
      Packet_Parameter2 = Packet_Parameter3;
      Packet_Parameter3 = Packet_Checksum;
      packet_position = 0;
      //ExitCritical();
      return false;
      break;

    default: //reset
      packet_position = 0;
      //ExitCritical();
      break;
  }

 // ExitCritical();
  return false;
}

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
void Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  OS_SemaphoreWait(PacketPutSemaphore, 0); //Wait on Packet Put Semaphore

  UART_OutChar(command); //Place Command byte in TxFIFO
  UART_OutChar(parameter1); //Place Parameter1 byte in TxFIFO
  UART_OutChar(parameter2); //Place Parameter2 byte in TxFIFO
  UART_OutChar(parameter3); //Place Parameter3 byte in TxFIFO
  UART_OutChar(command ^ parameter1 ^ parameter2 ^ parameter3); //Place Checksum byte in TxFIFO

  OS_SemaphoreSignal(PacketPutSemaphore); //Signal Packet Put Semaphore
}

/*! @brief Handles the stored packet
 *
 *  @return void
 */
void Packet_Handle(void)
{
  bool error = true; //used to determine whether an error has occurred

  //Mask out the Acknowledgment Bit from the Packet Command so that it can be processed
  switch (Packet_Command & ~PACKET_ACK_MASK)
  {
    case GET_STARTUP_VAL:
      //Place the Tower Startup packet in the TxFIFO
      Packet_Put(TOWER_STARTUP_COMM, TOWER_STARTUP_PAR1, TOWER_STARTUP_PAR2, TOWER_STARTUP_PAR3);

      //Place the Tower Version packet in the TxFIFO
      Packet_Put(TOWER_VERSION_COMM, TOWER_VERSION_V, TOWER_VERSION_MAJ, TOWER_VERSION_MIN);

      //Place the Tower Number packet in the TxFIFO
      Packet_Put(TOWER_NUMBER_COMM, TOWER_NUMBER_PAR1, TowerNumber->s.Lo, TowerNumber->s.Hi); //towerNumberLsb, towerNumberMsb))

      error = false;
      break;

    case GET_VERSION:
      //Place the Tower Version packet in the TxFIFO
      Packet_Put(TOWER_VERSION_COMM, TOWER_VERSION_V, TOWER_VERSION_MAJ, TOWER_VERSION_MIN);
      error = false;
      break;

    case TOWER_NUMBER:

      if (Packet_Parameter1 == TOWER_NUMBER_GET)
      {
	//Sub-Command: Get the Tower Number
	//Place the Tower Number packet in the TxFIFO
	Packet_Put(TOWER_NUMBER_COMM, TOWER_NUMBER_PAR1, TowerNumber->s.Lo, TowerNumber->s.Hi);
	error = false;

      } else if (Packet_Parameter1 == TOWER_NUMBER_SET)
      {
	//Sub-Command: Set the Tower Number
	uint16union_t temp;
	temp.s.Hi = Packet_Parameter3;
	temp.s.Lo = Packet_Parameter2;
	error = !Flash_Write16((uint16_t volatile *) TowerNumber, temp.l);
      }
      break;
    case GET_TOWER_MODE:
      if (Packet_Parameter1 == TOWER_MODE_GET)
      {
	//Sub-Command: Get Tower Mode
	Packet_Put(TOWER_MODE_COMM, TOWER_MODE_PAR1, TowerMode->s.Lo, TowerMode->s.Hi);
	error = false;
      }
      else if (Packet_Parameter1 == TOWER_MODE_SET)
      {
	//Sub-Command: Set Tower Mode (In flash)
	uint16union_t temp;
	temp.s.Hi = Packet_Parameter3;
	temp.s.Lo = Packet_Parameter2;
	error = !Flash_Write16((uint16_t volatile *) TowerMode, temp.l);
      }
      break;
    case FLASH_PROGRAM_BYTE:
      //if 8, erase flash, if >8, this is an error, < 8 refers to offset address in memory
      if(Packet_Parameter1 > 8) error = true;
      if(Packet_Parameter1 == 8) error = !Flash_Erase();
      else {
	uint8_t *tempAdr = (uint8_t *)(FLASH_DATA_START + Packet_Parameter1);
	error = !Flash_Write8((uint8_t volatile *) tempAdr, Packet_Parameter3);
      }
      break;
    case FLASH_READ_BYTE:
      //check that the address is within boundaries
      if (Packet_Parameter1 < 0 || Packet_Parameter1 > 7) error = true;
      else{
	//Access single byte of flash data
	uint8_t * const byte;
	*byte = _FB(FLASH_DATA_START + Packet_Parameter1);
	Packet_Put(TOWER_READ_BYTE_COMM, Packet_Parameter1, 0x0, *byte);
	error = false;
      }
      break;
    case CMD_CHARACTERISTIC:
      //set characteristic
      if(Packet_Parameter1 == 0X00 & Packet_Parameter2 == 0X01)
      {
          //Sub-Command: Get the Characteristic
          //Place the characteristic packet in the TxFIFO
//          Packet_Put(CHAR_COMM, CHAR_PAR1, ,);
//          error = false;
      }
      else if (Packet_Parameter1 == 0X00 & Packet_Parameter2 == 0X02)
      {

                if(Packet_Parameter3 == 0x01)
                  characteristic = INVERSE;
                else if(Packet_Parameter3 == 0x02)
                  characteristic = VERYINVERSE;
                else if(Packet_Parameter3 == 0x03)
                  characteristic = EXTREMELYINVERSE;

                 //WRITE TO FLASH
//          error = !Flash_Write8((uint16_t volatile *) charac, Packet_Parameter3);//Flash_Write16

      }
      break;
    default:
      break;
  }

  //Check whether the Acknowledgment bit is set
  if (Packet_Command & PACKET_ACK_MASK)
  {
    //Create a new command response packet
    uint8_t maskedPacket = 0;

    if (error == true)
    {
      //If there are errors, the Acknowledgment bit should be 0
      maskedPacket = Packet_Command & ~PACKET_ACK_MASK;
    } else
    {
      //If there are no errors, the Acknowledgment bit should be 1
      maskedPacket = Packet_Command | PACKET_ACK_MASK;
    }

    //Place the Acknowledgment Packet in the TxFIFO
    Packet_Put(maskedPacket, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  }
}

/*!
 * @}
 */
