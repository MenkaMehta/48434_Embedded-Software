/*! @file
 *
 * @brief implements functions to read, write and erase flash
 *
 * @author Corey Stidston & Menka Mehta
 * @date 2017-04-18
 */
/*!
 * @addtogroup flash_module Flash module documentation
 * @{
 */
/****************************************HEADER FILES****************************************************/
#include "types.h"
#include "Flash.h"
#include "MK70F12.h"
#include <string.h>

/****************************************GLOBAL VARS*****************************************************/

#define FCMD_ERASE_SEC 0x09LU
#define FCMD_PGM_PHRASE 0x07LU
#define FLASH_MAX_ADR (FLASH_DATA_END - FLASH_DATA_START)

typedef union
{
  uint32_t a;			//32 bit address in flash

  struct
  {
    uint8_t a0;		//Address [7:0] in flash
    uint8_t a8;		//Address [15:8] in flash
    uint8_t a16;	//Address [23:16] in flash
    uint8_t null;   //Not used..
  } ADR;

} FCCOB_ADR_t;

uint8_t phrase_alloc = 0xFF; //Represents the 8 bytes in flash memory and whether they have been allocated

/****************************************PRIVATE FUNCTION DECLARATION***********************************/
static bool WritePhrase(const uint64_t phrase);
static void WaitCCIF(void);
static void SetCCIF(void);
static bool ReadPhrase(uint64_t * const phrase);

/****************************************PUBLIC FUNCTION DEFINITION***************************************/

/*! @brief Enables the Flash module.
 *
 *  @return bool - TRUE if the Flash was setup successfully.
 */
bool Flash_Init(void)
{
  SIM_SCGC3 |= SIM_SCGC3_NFC_MASK;  	/* !Initialize the Flash Clock  */
  WaitCCIF();
  return true;
}

/*! @brief Allocates space for a non-volatile variable in the Flash memory.
 *
 *  @param variable is the address of a pointer to a variable that is to be allocated space in Flash memory.
 *         The pointer will be allocated to a relevant address:
 *         If the variable is a byte, then any address.
 *         If the variable is a half-word, then an even address.
 *         If the variable is a word, then an address divisible by 4.
 *         This allows the resulting variable to be used with the relevant Flash_Write function which assumes a certain memory address.
 *         e.g. a 16-bit variable will be on an even address
 *  @param size The size, in bytes, of the variable that is to be allocated space in the Flash memory. Valid values are 1, 2 and 4.
 *  @return bool - TRUE if the variable was allocated space in the Flash memory.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_AllocateVar(volatile void** variable, const uint8_t size)
{
  int mask;
  int addressPos;

  switch(size)
  {
    case 1:
      mask = 0x80; /* 10000000 */
      break;
    case 2:
      mask = 0xC0; /* 11000000 */
      break;
    case 4:
      mask = 0xF0; /* 11110000 */
      break;
  }

  /* based on size of variable (1 , 2 or 4 bytes)
   * it changes mask and how many bit shifts occur
   *
   */
  for(addressPos = FLASH_DATA_START; addressPos < (FLASH_DATA_END+1); addressPos += size)
  {
    if(mask == (phrase_alloc & mask)) 		//has this address been allocated?
    {
      *variable = (void *) addressPos;
      phrase_alloc = (phrase_alloc ^ mask); //alter the phrase allocation
      return true;
    }
    mask = mask >> size;						//shift the mask to check the next byte
  }
  return false;
}

/*! @brief Writes a 32-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 32-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 4-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{
  size_t index = ((size_t) address - FLASH_DATA_START); //index represents a flash address
  if (index < 0 || index > FLASH_MAX_ADR) return false;				//check that the index lies within parameters
  if (index % 4 != 0) return false;						//check that the index is valid
  index /= 4;											//index now represents an address for 32 bits
  uint64_t newPhrase;
  ReadPhrase(&newPhrase);								//read the existing phrase in flash
  uint32_t *psuedoArray = (uint32_t *) &newPhrase; 		//splits into two 4 byte segments
  psuedoArray[index] = data;							//alter the phrase at index position
  WritePhrase(newPhrase);								//write new phrase
  return true;
}

/*! @brief Writes a 16-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 16-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if address is not aligned to a 2-byte boundary or if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  size_t index = ((size_t) address - FLASH_DATA_START); //index represents a flash address
  if (index < 0 || index > FLASH_MAX_ADR) return false;				//check that the index lies within parameters
  if (index % 2 != 0) return false;						//check that the index is valid
  index /= 2;											//index now represents an address for 16 bits
  uint64_t newPhrase;
  ReadPhrase(&newPhrase);								//read the existing phrase in flash
  uint16_t *psuedoArray = (uint16_t *) &newPhrase; 		//splits into four 2 byte segments
  psuedoArray[index] = data;							//alter the phrase at index position
  WritePhrase(newPhrase);								//write new phrase
  return true;
}

/*! @brief Writes an 8-bit number to Flash.
 *
 *  @param address The address of the data.
 *  @param data The 8-bit data to write.
 *  @return bool - TRUE if Flash was written successfully, FALSE if there is a programming error.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{
  size_t index = ((size_t) address - FLASH_DATA_START); //index represents a flash address
  if (index > FLASH_MAX_ADR || index < 0)							//check that the index lies within parameters
  {
    return false;
  }
  uint64_t newPhrase;
  ReadPhrase(&newPhrase);								//read the existing phrase in flash
  uint8_t *psuedoArray = (uint8_t *) &newPhrase; 		//splits into eight 1 byte segments
  psuedoArray[index] = data;							//alter the phrase at index position
  WritePhrase(newPhrase);								//write new phrase
  return true;
}

/*! @brief Erases the entire Flash sector.
 *
 *  @return bool - TRUE if the Flash "data" sector was erased successfully.
 *  @note Assumes Flash has been initialized.
 */
bool Flash_Erase(void)
{
  FCCOB_ADR_t fccob;

  //phrase_alloc = 0xFF;
  WaitCCIF();

  fccob.a = FLASH_DATA_START;
  FTFE_FCCOB0 = FCMD_ERASE_SEC; // FTFE command to erase sector
  FTFE_FCCOB1 = fccob.ADR.a16; 	// sets flash address[23:16]
  FTFE_FCCOB2 = fccob.ADR.a8; 	// sets flash address[15:8]
  FTFE_FCCOB3 = fccob.ADR.a0;	// sets flash address[8:0]

  SetCCIF();
  WaitCCIF();

  // errors pg 783/784 K70 manual
  return true; //Later on, need to check error flags
}

/****************************************PRIVATE FUNCTION DEFINITION***************************************/
/*! @brief Reads the phrase starting from FLASH_DATA_START
 *
 *	@param phrase Pointer returned to the phrase
 *  @return bool - TRUE if the the phrase was read
 *  @note Assumes Flash has been initialized.
 */
bool ReadPhrase(uint64_t * const phrase)
{
  WaitCCIF();
  *phrase = _FP(FLASH_DATA_START);
  return true;
}

/*! @brief Writes phrase
 *
 *  @param phrase
 *
 *  @return bool - TRUE
 *
 */
//P 789 and P806
bool WritePhrase(const uint64_t phrase) //const uint32_t address,
{
  uint8_t *bytes = (uint8_t *) &phrase; //split into eight 1 byte segments for saving to flash
  FCCOB_ADR_t fccob;

  WaitCCIF();

  if(!Flash_Erase())			//Erase flash before writing as per manual
  {
    return false;
  }

  fccob.a = FLASH_DATA_START;
  FTFE_FCCOB0 = FCMD_PGM_PHRASE; // defines the FTFE command to write
  FTFE_FCCOB1 = fccob.ADR.a16; 	 // sets flash address[23:16]
  FTFE_FCCOB2 = fccob.ADR.a8; 	 // sets flash address[15:8]
  FTFE_FCCOB3 = fccob.ADR.a0; 	 // sets flash address[8:0]

  //Switched/Sorted for Big Endian
  FTFE_FCCOB4 = bytes[3];
  FTFE_FCCOB5 = bytes[2];
  FTFE_FCCOB6 = bytes[1];
  FTFE_FCCOB7 = bytes[0];
  FTFE_FCCOB8 = bytes[7];
  FTFE_FCCOB9 = bytes[6];
  FTFE_FCCOBA = bytes[5];
  FTFE_FCCOBB = bytes[4];

  SetCCIF(); //Initiates the command
  WaitCCIF();//Waits until program can continue to execute

  return true;
}

/* @brief Wait for the CCIF register to be set to 1.
 *
 */
void WaitCCIF(void)
{
  //(https://community.nxp.com/thread/329360)
  //wait for the command to complete.
  while (!(FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK));

  //this waits until CCIF register is set to 1
}

/*! @brief Set CCIF and the wait for it to be set.
 * Used to start a flash command and wait for it to complete
 *
 */
void SetCCIF(void)
{
  FTFE_FSTAT |= FTFE_FSTAT_CCIF_MASK;
}

//All required FCCOBx registers are written, so launch the command
// This line is occurred ACCERR.
//  FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK;
//pg 807 K70 Manual
// Before launching a command, the ACCERR and FPVIOL bits in the FSTAT register
// must be zero and the CCIF flag must read 1 to verify that any previous command has
// completed. If CCIF is zero, the previous command execution is still active, a new
// command write sequence cannot be started, and all writes to the FCCOB registers are
// ignored.

/*!
 * @}
 */
