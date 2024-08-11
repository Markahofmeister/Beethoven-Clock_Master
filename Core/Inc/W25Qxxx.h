/*
 * W25Qxxx.h
 *
 *  Created on: Aug 11, 2024
 *      Author: marka
 */

#ifndef INC_W25QXXX_H_
#define INC_W25QXXX_H_

#ifndef  STM32G0XX_HAL_H_
#include "stm32g0xx_hal.h"
#endif

/*
 * Command table
 */
#define CMD_WRITE_ENABLE 		0x06
#define CMD_VOL_SR_WE 			0x50
#define CMD_WRITE_DISABLE		0x04
#define CMD_READ_RELEASE_ID		0xAB
#define CMD_READ_MFR_ID			0x90
#define CMD_READ_JEDEC_ID		0x9F
#define CMD_READ_UID			0x4B
#define CMD_READ_DATA			0x03
#define CMD_FAST_READ			0x0B
#define CMD_PAGE_PROG			0x02
#define CMD_SECT_ERASE_4KB		0x20
#define CMD_BLOCK_ERASE_32KB	0x52
#define CMD_BLOCK_ERASE_64KB	0xD8
#define CMD_CHIP_ERASE			0x60
#define CMD_READ_STAT_1			0x05
#define CMD_READ_STAT_2			0x35
#define CMD_READ_STAT_3			0x15
#define CMD_WRITE_STAT_1		0x01
#define CMD_WRITE_STAT_2		0x31
#define CMD_WRITE_STAT_3		0x11
#define CMD_READ_SFDP			0x5A
#define CMD_ERASE_SEC_REG		0x44
#define CMD_PROG_SEC_REG		0x42
#define CMD_READ_SEC_REG		0x48
#define CMD_GLOB_BLOCK_LOCK		0x7E
#define CMD_GLOB_BLOCK_UNLOCK	0x98
#define CMD_READ_BLOCK_LOCK		0x3D
#define CMD_IND_BLOCK_LOCK		0x36
#define CMD_IND_BLOCK_UNLOCK	0x39
#define CMD_PROG_SUSPEND		0x75
#define CMD_PROG_RESUME			0x7A
#define CMD_POWER_DOWN			0xB9
#define CMD_RESET_ENABLE		0x66
#define CMD_RESET_DEVICE		0x99


/*
 * Struct to serve as object for memory chip
 */
typedef struct {

	/*
	 * GPIO Ports for:
	 * 	- SPI Chip Select Pin
	 * 	- Write Protect Pin
	 * 	- Reset Pin
	 */
	GPIO_TypeDef *nCSPort;
	GPIO_TypeDef *nWPPort;
	GPIO_TypeDef *nRSTPort;

	/*
	 * GPIO Pin mapping for:
	 * - SPI Chip Select Pin
	 * - Write Protect Pin
	 * - Reset Pin
	 */
	uint32_t nCSPin;
	uint32_t nWPPin;
	uint32_t nRSTPin;

	// STM32 handle for SPI bus
	SPI_HandleTypeDef *hspi;


} W25Q;

uint8_t W_25Q_Init(GPIO_TypeDef *nCSPort, GPIO_TypeDef *nWPPort, GPIO_TypeDef *nRSTPort,
					uint32_t nCSPin, uint32_t nWPPin, uint32_t nRSTPin, SPI_HandleTypeDef *hspi);





#endif /* INC_W25QXXX_H_ */
