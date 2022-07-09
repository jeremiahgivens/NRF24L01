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
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN , GPIO_PIN_SET);
}

void CSN_Low(void){
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN , GPIO_PIN_RESET);
}

//Write a single byte to a register
void nrf24_WriteReg(uint8_t Reg, uint8_t Data){

	uint8_t buf[2];
	buf[0] = Reg | (1 << 5);
	buf[1] = Data;

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, (uint8_t *)&buf, 2, 1000);
	//Set CSN pin high
	CSN_High();
}

//Write multiple bytes to a register
void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *Data, int Size){

	uint8_t buf = Reg | (1 << 5);

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, (uint8_t *)&buf, 1, 1000);
	HAL_SPI_Transmit(NRF24_SPI, (uint8_t *)&Data, Size, 1000);
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
	HAL_SPI_Transmit(NRF24_SPI, (uint8_t *)&Reg, 1, 1000);
	HAL_SPI_Receive(NRF24_SPI, (uint8_t *)&Data, Size, 1000);
	//Set CSN pin high
	CSN_High();
}

void nrf_sendCmd(uint8_t cmd){

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, (uint8_t *)&cmd, 1, 1000);
	//Set CSN pin high
	CSN_High();
}

//Setup NRF device
void NRF24_Init(void){

	//disable the device before writing to config register
	CE_Low();

	nrf24_WriteReg(CONFIG, 0); //Will configure when setting as reciever or transmitter
	nrf24_WriteReg(EN_AA, 0); //No auto acknowledgment
	nrf24_WriteReg(EN_RXADDR, 0); //Leaving all data pipes off for now
	nrf24_WriteReg(SETUP_AW, 0x03); //5 Byte address for TX and RX
	nrf24_WriteReg(SETUP_RETR, 0x00); //Disabling automatic retransmission
	nrf24_WriteReg(RF_CH, 0); //Will setup when determining RX or TX
	nrf24_WriteReg(RF_SETUP, 0x0E); //Power 0dB, data rate 2Mbps

	CE_High();
}

void NRF24_TxMode(uint8_t *Address, uint8_t channel){
	CE_Low();

	nrf24_WriteReg(RF_CH, channel); //Select channel
	nrf24_WriteRegMulti(SETUP_AW, Address,  5); //Set address

	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1 << 1);

	nrf24_WriteReg(CONFIG, config); //Power up device


	CE_High();
}

uint8_t NRF24_Transmit(uint8_t *Data){
	CE_Low();

	uint8_t cmd = W_TX_PAYLOAD;

	//Now we pull the CSN pin low
	CSN_Low();
	//Send the data
	HAL_SPI_Transmit(NRF24_SPI, (uint8_t *)cmd, 1, 1000);
	HAL_SPI_Transmit(NRF24_SPI, Data, 32, 1000);
	//Set CSN pin high
	CSN_High();

	CE_High();

	HAL_Delay(1);

	uint8_t fifoStatus = nrf24_ReadReg(FIFO_STATUS);

	uint8_t str[100];
	sprintf(str, "fifoStatus = %02X\n", fifoStatus);
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 1000);

	if((fifoStatus&(1<<4)) && !(fifoStatus&(1<<3))){
		nrf_sendCmd(FLUSH_TX);
		return 1;
	}

	return 0;
}

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
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 1000);
	*/

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







