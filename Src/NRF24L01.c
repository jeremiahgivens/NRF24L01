/*
 * NRF24L01.c
 *
 *  Created on: Jul 3, 2022
 *      Author: jeremiahgivens
 */


#include "stm32f0xx_hal.h"
#include "NRF24L01.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;
#define NRF24_SPI &hspi2

extern UART_HandleTypeDef huart1;

#define NRF24_CE_PORT GPIOB
#define NRF24_CE_PIN GPIO_PIN_1

#define NRF24_CSN_PORT GPIOB
#define NRF24_CSN_PIN GPIO_PIN_2

void CE_High(void){
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN , GPIO_PIN_SET);
}

void CE_Low(void){
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN , GPIO_PIN_RESET);
}

void CSN_High(void){
	HAL_Delay(1);
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN , GPIO_PIN_SET);
	HAL_Delay(1);
}

void CSN_Low(void){
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN , GPIO_PIN_RESET);
	HAL_Delay(1);
}

//Write a single byte to a register
void nrf24_WriteReg(uint8_t Reg, uint8_t Data){

	uint8_t buf[2];
	buf[0] = Reg |1<<5;
	buf[1] = Data;

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);
	//Set CSN pin high
	CSN_High();
}

//Write multiple bytes to a register
void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *Data, int Size){

    uint8_t buf[2];
    buf[0] = Reg|1<<5;

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 1000);
	HAL_SPI_Transmit(NRF24_SPI, Data, Size, 1000);
	//Set CSN pin high
	CSN_High();
}

//Read a single byte from a register
uint8_t nrf24_ReadReg(uint8_t Reg){

	uint8_t buf = 0;

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, (uint8_t *)&Reg, 1, 1000);
	HAL_SPI_Receive(NRF24_SPI, (uint8_t *)&buf, 1, 1000);
	//Set CSN pin high
	CSN_High();

	return buf;
}

//Read multiple bytes from a register
void nrf24_ReadRegMulti(uint8_t Reg, uint8_t *Data, int Size){

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 1000);
	HAL_SPI_Receive(NRF24_SPI, Data, Size, 1000);
	//Set CSN pin high
	CSN_High();
}

void nrf_sendCmd(uint8_t cmd){

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 1000);
	//Set CSN pin high
	CSN_High();
}

void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		nrf24_WriteReg(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
	nrf24_WriteReg(CONFIG, 0x08);
	nrf24_WriteReg(EN_AA, 0x3F);
	nrf24_WriteReg(EN_RXADDR, 0x03);
	nrf24_WriteReg(SETUP_AW, 0x03);
	nrf24_WriteReg(SETUP_RETR, 0x03);
	nrf24_WriteReg(RF_CH, 0x02);
	nrf24_WriteReg(RF_SETUP, 0x0E);
	nrf24_WriteReg(STATUS, 0x00);
	nrf24_WriteReg(OBSERVE_TX, 0x00);
	nrf24_WriteReg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_WriteReg(RX_ADDR_P2, 0xC3);
	nrf24_WriteReg(RX_ADDR_P3, 0xC4);
	nrf24_WriteReg(RX_ADDR_P4, 0xC5);
	nrf24_WriteReg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
	nrf24_WriteReg(RX_PW_P0, 0);
	nrf24_WriteReg(RX_PW_P1, 0);
	nrf24_WriteReg(RX_PW_P2, 0);
	nrf24_WriteReg(RX_PW_P3, 0);
	nrf24_WriteReg(RX_PW_P4, 0);
	nrf24_WriteReg(RX_PW_P5, 0);
	nrf24_WriteReg(FIFO_STATUS, 0x11);
	nrf24_WriteReg(DYNPD, 0);
	nrf24_WriteReg(FEATURE, 0);
	}
}



// Setup the device
void NRF24_Init (void)
{
	// disable the chip before configuring the device
	CE_Low();


	// reset everything
	nrf24_reset (0);

	nrf24_WriteReg(CONFIG, 0);  // will be configured later

	nrf24_WriteReg(EN_AA, 0);  // No Auto ACK

	nrf24_WriteReg (EN_RXADDR, 0);  // Not Enabling any data pipe right now

	nrf24_WriteReg (SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address

	nrf24_WriteReg (SETUP_RETR, 0);   // No retransmission

	nrf24_WriteReg (RF_CH, 0);  // will be setup during Tx or RX

	nrf24_WriteReg (RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps

	// Enable the chip after configuring the device
	CE_High();

}

void NRF24_TxMode(uint8_t *Address, uint8_t channel){
	CE_Low();

	nrf24_WriteReg(RF_CH, channel); //Select channel
	nrf24_WriteRegMulti(TX_ADDR, Address,  5); //Set address

	uint8_t config = nrf24_ReadReg(CONFIG);
	config = 0xF2;    // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked;


	nrf24_WriteReg(CONFIG, config); //Power up device


	CE_High();
}

uint8_t NRF24_Transmit (uint8_t *data)
{

    uint8_t message[100];
    strcpy((char*)message, "Entering the NRF24_Transmit function.\n");
    //HAL_UART_Transmit(&huart1, message, strlen(message), 1000);

    uint8_t cmdtosend = 0;
    uint8_t cmd[33];

    // select the device
    CSN_Low();

    // payload command
    cmd[0] = W_TX_PAYLOAD;
    memcpy(&cmd[1], data, 32);
    //HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

    // send the payload
    //HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);
    HAL_SPI_Transmit(NRF24_SPI, &cmd, 33, 1000);

    // Unselect the device
    CSN_High();

    HAL_Delay(1);



    //nrf24_reset (FIFO_STATUS);
    uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

    /*

    sprintf(message, "fifostatus = %02X.\n", fifostatus);
    HAL_UART_Transmit(&huart1, message, strlen(message), 1000);

        nrf24_reset (STATUS);
        uint8_t status = nrf24_ReadReg(STATUS);

        sprintf(message, "status = %02X.\n", status);
        HAL_UART_Transmit(&huart1, message, strlen(message), 1000);
        */

    // check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
    if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
    {
        cmdtosend = FLUSH_TX;
        nrf_sendCmd(cmdtosend);

        // reset FIFO_STATUS
        nrf24_reset (FIFO_STATUS);

        return 1;
    }

    return 0;
}


/*
uint8_t NRF24_Transmit(uint8_t *Data){

	uint8_t cmd[33];

	//Now we pull the CSN pin low
	CSN_Low();

	cmd[0] = W_TX_PAYLOAD;
	memcpy(&cmd[1], Data, 32);
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 33, 1000);
	//Set CSN pin high
	CSN_High();

	HAL_Delay(1);

	uint8_t fifoStatus = nrf24_ReadReg(FIFO_STATUS);

	uint8_t str[100];
	sprintf(str, "fifoStatus = %02X\n", fifoStatus);
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 1000);


	if((fifoStatus&(1<<4)) && !(fifoStatus&(1<<3))){
		uint8_t cmdToSend = FLUSH_TX;
		nrf_sendCmd(cmdToSend);
		return 1;
	}

	return 0;
}

*/

void NRF24_RxMode(uint8_t *Address, uint8_t channel){
	CE_Low();

	nrf24_WriteReg(RF_CH, channel); //Select channel

	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR) | 0x01;

	nrf24_WriteReg(EN_RXADDR, en_rxaddr);

	nrf24_WriteRegMulti(RX_ADDR_P0, Address,  5); //Set address

	nrf24_WriteReg(RX_PW_P0, 32);

	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1 << 1) | 0x01;

	nrf24_WriteReg(CONFIG, config); //Power up device

	CE_High();
}

uint8_t isDataAvailable(int pipNum){
	uint8_t status = nrf24_ReadReg(STATUS);

    /*
	uint8_t str[100];
	sprintf(str, "status = %02X\n", status);
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 1000); */


	if ((status & (1<< 6))){

		nrf24_WriteReg(STATUS, (1<<6));

		return 1;
	}

	return 0;
}

void NRF24_Receive(uint8_t *Data){

	uint8_t cmd = R_RX_PAYLOAD;

	CE_Low();

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 1000);
	HAL_SPI_Receive(NRF24_SPI, Data, 32, 1000);
	//Set CSN pin high
	CSN_High();

	HAL_Delay(1);

	nrf_sendCmd(FLUSH_RX);

	CE_High();
}







