/*
 * W25Qxxx.c
 *
 *  Created on: Aug 11, 2024
 *      Author: marka
 */

#ifndef W25QXXX_H_
#include "../Inc/W25Qxxx.h"
#endif


uint8_t W25Q_Init(W25Q *wq, GPIO_TypeDef *nCSPort, GPIO_TypeDef *nWPPort, GPIO_TypeDef *nRSTPort,
					uint32_t nCSPin, uint32_t nWPPin, uint32_t nRSTPin, SPI_HandleTypeDef *hspi, uint8_t devID) {

	// Map struct GPIO Ports & Pins to passed parameters
	wq->nCSPort = nCSPort;
	wq->nWPPort = nWPPort;
	wq->nRSTPort = nRSTPort;

	wq->nCSPin = nCSPin;
	wq->nWPPin = nWPPin;
	wq->nRSTPin = nRSTPin;

	wq->hspi = hspi;

	// HAL status handle used to indicate error messages
	HAL_StatusTypeDef halRet = HAL_OK;

	// TODO: Toggle GPIOs properly

	// Get + Store IDs
	halRet = W25Q_GetIDs(wq, devID);
	if(halRet != HAL_OK)
		return 1;

	// TODO: Disable Write Access to Memory

	// TODO: Get + Store Status Register Contents, check WP0




	return 0;

}

/*
 * Will release device from power-down and fetch all of the IDs
 */
HAL_StatusTypeDef W25Q_GetIDs(W25Q *wq, uint8_t devID_passed) {


	HAL_StatusTypeDef halRet = HAL_OK;

	/*
	 * Release power down + read device ID
	 */
		// TX release power-down and return device ID instruction
		halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[8]){CMD_READ_RELEASE_ID, 0x00, 0x00, 0x00}, 4, HAL_MAX_DELAY);
		if(halRet != HAL_OK)
			return halRet;

		// RX device ID into struct variable
		halRet = HAL_SPI_Receive(wq->hspi, &(wq->devID), 1, HAL_MAX_DELAY);

		// Check to ensure that received device ID is equal to that passed by the user in the init function
		if((halRet != HAL_OK) || (wq->devID != devID_passed))
			return halRet;

		wq->powerUp = 1; 		// We have now successfully exited power-down state

	/*
	 * Read and check MFR ID
	 */
		// TX read mfr + device ID command
		halRet = HAL_SPI_Transmit(wq->hspi, (uint8_t[8]){CMD_READ_MFR_ID, 0x00, 0x00, 0x00}, 4, HAL_MAX_DELAY);
		if(halRet != HAL_OK)
			return halRet;

		// RX device and mfr ID, store in struct variable
		uint8_t retIDs[2] = {0x00, 0x00};
		halRet = HAL_SPI_Receive(wq->hspi, retIDs, 1, HAL_MAX_DELAY);
		wq->mfrID = retIDs[0];

		// Check to ensure that received mfr ID is correct.
		if((halRet != HAL_OK) || (wq->mfrID != 0xEF))
			return halRet;

	/*
	 * Leaving out for now: Read JEDEC ID
	 */

		return halRet;
}

HAL_StatusTypeDef W25Q_EnableWrite(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}

HAL_StatusTypeDef W25Q_DisableWrite(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}

HAL_StatusTypeDef W25Q_ReadStatusRegs(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}

HAL_StatusTypeDef W25Q_WriteStatusReg1(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}

HAL_StatusTypeDef W25Q_WriteStatusReg2(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}

HAL_StatusTypeDef W25Q_WriteStatusReg3(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}

HAL_StatusTypeDef W25Q_readData(W25Q *wq, uint32_t startAddress, uint32_t dataSize, uint8_t *dataLocation) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}

HAL_StatusTypeDef W25Q_ChipErase(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}

HAL_StatusTypeDef W25Q_ChipPowerDown(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;
}

HAL_StatusTypeDef W25Q_ChipReset(W25Q *wq) {

	HAL_StatusTypeDef halRet = HAL_OK;

	return halRet;

}
