/*
 * FLASH_SECTOR.h
 *
 *  Created on: May 15, 2020
 *      Author: controllerstech
 */

#ifndef INC_FLASH_SECTOR_H_
#define INC_FLASH_SECTOR_H_

#define START_F_ADDRESS_IP 0x08060000

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t * DATA_32, uint32_t numberofwords);
void Flash_Read_Data (uint32_t StartPageAddress, __IO uint32_t * DATA_32, uint32_t numberofwords);
void Convert_To_Str (uint32_t *data, char *str);


#endif /* INC_FLASH_SECTOR_H_ */
