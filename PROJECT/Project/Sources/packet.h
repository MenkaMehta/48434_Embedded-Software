/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author PMcL
 *  @date 2015-07-23
 */
/*!
 * @addtogroup packet_module packet documentation
 * @{
 */

#ifndef PACKET_H
#define PACKET_H

// New types
#include "types.h"
#include "OS.h"

OS_ECB *PacketPutSemaphore; //Semaphore for Packet Put

// Packet structure
#define PACKET_NB_BYTES 5

#pragma pack(push)
#pragma pack(1)

typedef union
{
  uint8_t bytes[PACKET_NB_BYTES];     /*!< The packet as an array of bytes. */
  struct
  {
    uint8_t command;		      /*!< The packet's command. */
    union
    {
      struct
      {
	uint8_t parameter1;	      /*!< The packet's 1st parameter. */
	uint8_t parameter2;	      /*!< The packet's 2nd parameter. */
	uint8_t parameter3;	      /*!< The packet's 3rd parameter. */
      } separate;
      struct
      {
	uint16_t parameter12;         /*!< Parameter 1 and 2 concatenated. */
	uint8_t parameter3;
      } combined12;
      struct
      {
	uint8_t paramater1;
	uint16_t parameter23;         /*!< Parameter 2 and 3 concatenated. */
      } combined23;
    } parameters;
    uint8_t checksum;
  } packetStruct;
} TPacket;

#pragma pack(pop)

#define Packet_Command     Packet.packetStruct.command
#define Packet_Parameter1  Packet.packetStruct.parameters.separate.parameter1
#define Packet_Parameter2  Packet.packetStruct.parameters.separate.parameter2
#define Packet_Parameter3  Packet.packetStruct.parameters.separate.parameter3
#define Packet_Parameter12 Packet.packetStruct.parameters.combined12.parameter12
#define Packet_Parameter23 Packet.packetStruct.parameters.combined23.parameter23
#define Packet_Checksum    Packet.packetStruct.checksum

/*************************************************PC TO TOWER COMMANDS*************************************************/

//The PC will issue this command upon startup to retrieve the state of the Tower to update the interface application.
#define GET_STARTUP_VAL 0x04

//Get the version of the tower software
#define GET_VERSION 0x09

#define GET_TOWER_MODE 0xd

#define TOWER_MODE_GET 1

#define TOWER_MODE_SET 2

#define FLASH_PROGRAM_BYTE 0x7

#define FLASH_READ_BYTE 0x8

#define SET_TIME 0x0C

//Get or set the tower number
#define TOWER_NUMBER 0x0B

//Packet Parameter 1 for getting the tower number
#define TOWER_NUMBER_GET 1

//Packet Parameter 1 for setting the tower number
#define TOWER_NUMBER_SET 2

//Least significant byte of Student ID
#define S_ID 0x13A8

/*************************************************TOWER TO PC COMMANDS*************************************************/

/*
 * The tower will issue this command upon startup to allow the PC to update
 * the interface application and the Tower.
 * Typically setup data will also be sent from the Tower to the PC.
 */
#define TOWER_STARTUP_COMM 0x04
#define TOWER_STARTUP_PAR1 0x0
#define TOWER_STARTUP_PAR2 0x0
#define TOWER_STARTUP_PAR3 0x0

//Get the tower version command
#define TOWER_VERSION_COMM 0x09
#define TOWER_VERSION_V 'v'
#define TOWER_VERSION_MAJ 1
#define TOWER_VERSION_MIN 0

//Get the tower number command
#define TOWER_NUMBER_COMM 0x0B
#define TOWER_NUMBER_PAR1 1

#define TOWER_MODE_COMM 0x0d
#define TOWER_MODE_PAR1 0x01

#define TOWER_READ_BYTE_COMM 0x08

extern TPacket Packet;

// Acknowledgment bit mask
extern const uint8_t PACKET_ACK_MASK;

//extern uint8_t towerNumberLsb, towerNumberMsb;
extern uint16union_t volatile *TowerNumber, *TowerMode;

/*************************************************PUBLIC FUNCTION DECLARATION*************************************************/

/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz.
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk);

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void);

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 */
void Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3);

/*! @brief Handles a packet once it has been validated by Packet_Get
 *
 *  @return void
 */
void Packet_Handle(void);

/*!
 * @}
 */
#endif
