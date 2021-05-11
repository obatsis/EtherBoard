/*
 * FLASH_SECTOR.c
 *
 *  Created on: May 15, 2020
 *      Author: controllerstech
 */


#include "stm32f4xx_hal.h"
#include "FLASH_SECTOR.h"
#include "string.h"
#include "stdio.h"

/**
  * @brief  Gets the sector of a given address
  * @param  Address, the address in flash memory
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address) {
  uint32_t sector = 0;

  if((Address < 0x08003FFF) && (Address >= 0x08000000)) {
    sector = FLASH_SECTOR_0;
  }
  else if((Address <  0x08007FFF) && (Address >= 0x08004000)) {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < 0x0800BFFF) && (Address >= 0x08008000)) {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < 0x0800FFFF) && (Address >= 0x0800C000)) {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < 0x0801FFFF) && (Address >= 0x08010000)) {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < 0x0803FFFF) && (Address >= 0x08020000)) {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < 0x0805FFFF) && (Address >= 0x08040000)) {
    sector = FLASH_SECTOR_6;
  }
  else if((Address <  0x0807FFFF) && (Address >= 0x08060000)) {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < 0x0809FFFF) && (Address >= 0x08080000)) {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < 0x080BFFFF) && (Address >= 0x080A0000)) {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < 0x080DFFFF) && (Address >= 0x080C0000)) {
    sector = FLASH_SECTOR_10;
  }
  else if((Address < 0x080FFFFF) && (Address >= 0x080E0000)) {
    sector = FLASH_SECTOR_11;
  }
 /* else if((Address < 0x08103FFF) && (Address >= 0x08100000))
  {
    sector = FLASH_SECTOR_12;
  }
  else if((Address < 0x08107FFF) && (Address >= 0x08104000))
  {
    sector = FLASH_SECTOR_13;
  }
  else if((Address < 0x0810BFFF) && (Address >= 0x08108000))
  {
    sector = FLASH_SECTOR_14;
  }
  else if((Address < 0x0810FFFF) && (Address >= 0x0810C000))
  {
    sector = FLASH_SECTOR_15;
  }
  else if((Address < 0x0811FFFF) && (Address >= 0x08110000))
  {
    sector = FLASH_SECTOR_16;
  }
  else if((Address < 0x0813FFFF) && (Address >= 0x08120000))
  {
    sector = FLASH_SECTOR_17;
  }
  else if((Address < 0x0815FFFF) && (Address >= 0x08140000))
  {
    sector = FLASH_SECTOR_18;
  }
  else if((Address < 0x0817FFFF) && (Address >= 0x08160000))
  {
    sector = FLASH_SECTOR_19;
  }
  else if((Address < 0x0819FFFF) && (Address >= 0x08180000))
  {
    sector = FLASH_SECTOR_20;
  }
  else if((Address < 0x081BFFFF) && (Address >= 0x081A0000))
  {
    sector = FLASH_SECTOR_21;
  }
  else if((Address < 0x081DFFFF) && (Address >= 0x081C0000))
  {
    sector = FLASH_SECTOR_22;
  }
  else if (Address < 0x081FFFFF) && (Address >= 0x081E0000)
  {
    sector = FLASH_SECTOR_23;
  }*/
  return sector;
}

/**
  * @brief  Gets sector Size
  * @param  Sector, 32 bit value of the sector
  * @param  banks, the number of memory banks (1 or 2)
  * @retval The size of a given sector
  */
//static uint32_t GetSectorSize(uint32_t Sector, uint8_t banks) {
//  uint32_t sectorsize = 0x00;
//  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) ||  (Sector == FLASH_SECTOR_3)) {
//    if(banks == 1) {
//    	sectorsize = 32 * 1024;
//    }
//    else if(banks == 2) {
//    	sectorsize = 16 * 1024;
//    }
//  }
//  else if((Sector == FLASH_SECTOR_4)) {
//    if(banks == 1) {
//    	sectorsize = 128 * 1024;
//    }
//    else if(banks == 2) {
//    	sectorsize = 64 * 1024;
//    }
//  }
//  else if((Sector == FLASH_SECTOR_5) || (Sector == FLASH_SECTOR_6) || (Sector == FLASH_SECTOR_7)|| (Sector == FLASH_SECTOR_8)|| (Sector == FLASH_SECTOR_9)|| (Sector == FLASH_SECTOR_10)|| (Sector == FLASH_SECTOR_11)) {
//    if(banks == 1) {
//    	sectorsize = 256 * 1024;
//    }
//    else if(banks == 2) {
//    	sectorsize = 128 * 1024;
//    }
//  }
//  // Uncomment below, if the device have more than 11 SECTORS
//
//  else if ((Sector == FLASH_SECTOR_12) || (Sector == FLASH_SECTOR_13) || (Sector == FLASH_SECTOR_14) || (Sector == FLASH_SECTOR_15)) {
//	sectorsize = 16 * 1024;
//  }
//
//  else if ((Sector == FLASH_SECTOR_16)) {
//	sectorsize = 64 * 1024;
//  }
//  else if ((Sector == FLASH_SECTOR_17) || (Sector == FLASH_SECTOR_18) || (Sector == FLASH_SECTOR_19)|| (Sector == FLASH_SECTOR_20)|| (Sector == FLASH_SECTOR_21)|| (Sector == FLASH_SECTOR_22)|| (Sector == FLASH_SECTOR_23)) {
//	sectorsize = 128 * 1024;
//  }
//  else {
//    sectorsize = 128 * 1024;
//  }
//
//  return sectorsize;
//}



uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t * DATA_32, uint32_t numberofwords) {

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	int sofar=0;

//	int numberofwords = (strlen(DATA_32)/4) + ((strlen(DATA_32) % 4) != 0);

//	int numberofwords = ((sizeof(DATA_32)/sizeof(uint32_t *))/4) + (((sizeof(DATA_32)/sizeof(uint32_t *)) % 4) != 0);

	 /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Erase the user Flash area */

	  /* Get the number of sector to erase from 1st sector */

	  uint32_t StartSector = GetSector(StartSectorAddress);
	  uint32_t EndSectorAddress = StartSectorAddress + numberofwords*4;
	  uint32_t EndSector = GetSector(EndSectorAddress);

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Sector        = StartSector;
	  EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;

	  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	     you have to make sure that these data are rewritten before they are accessed during code
	     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	     DCRST and ICRST bits in the FLASH_CR register. */
	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {
		  return HAL_FLASH_GetError ();
	  }

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	   while (sofar<numberofwords) {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, DATA_32[sofar]) == HAL_OK) {
	    	 StartSectorAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	   return 0;
}

//@todo
// here we can add another parameter to specify the length of the data we are trying to read, so we would not need the while anymore
void Flash_Read_Data (uint32_t StartSectorAddress, __IO uint32_t * DATA_32, uint32_t numberofwords) {

	for(uint32_t i = 0; i < numberofwords; i++, StartSectorAddress += 4, DATA_32++) {
		*DATA_32 = *(__IO uint32_t *)StartSectorAddress;
	}
}

//void Convert_To_Str (uint32_t *data, char *str) {
//	int numberofbytes = ((strlen(data)/4) + ((strlen(data) % 4) != 0)) *4;
////	int numberofbytes = (((sizeof(data)/sizeof(uint32_t *))/4) + (((sizeof(data)/sizeof(uint32_t *)) % 4) != 0)) *4;
//
//	for (int i=0; i<numberofbytes; i++) {
//		str[i] = data[i/4]>>(8*(i%4));
//	}
//}