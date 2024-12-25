#include "main.h"
#include "spi.h"
#include <stdio.h>
#include <stdlib.h>

#include "NRF24.h"
#include "NRF24_Defs.h"

static SPI_HandleTypeDef *hspi_nrf;

static uint8_t addr_p0_backup[NRF24_ADDR_SIZE];

extern volatile uint8_t nrf24_rx_flag, nrf24_tx_flag, nrf24_mr_flag;
extern uint8_t NRF24_Tx_Inte, NRF24_Rx_Inte;

//
// BASIC READ/WRITE FUNCTIONS
//
// Define these function for your MCU
//

#define NRF24_CSN_HIGH		HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET)
#define NRF24_CSN_LOW		HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET)

#define NRF24_CE_HIGH		HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_SET)
#define NRF24_CE_LOW		HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET)

static void nRF24_Delay(uint8_t Time)
{
	HAL_Delay(1);
}

static void nRF24_SendSpi(uint8_t *Data, uint8_t Length)
{
	HAL_SPI_Transmit(hspi_nrf, Data, Length, 1000);
}
static void nRF24_SendSpi_IT(uint8_t *Data, uint8_t Length)
{
	HAL_SPI_Transmit_IT(hspi_nrf, Data, Length);
}


static void nRF24_ReadSpi(uint8_t *Data, uint8_t Length)
{
	HAL_SPI_Receive(hspi_nrf, Data, Length, 1000);
}
static void nRF24_ReadSpi_IT(uint8_t *Data, uint8_t Length)
{
	HAL_SPI_Receive_IT(hspi_nrf, Data, Length);
}

//
// END OF BASIC READ/WRITE FUNCTIONS
//

static uint8_t nRF24_ReadRegister(uint8_t reg)
{
	uint8_t result;

	reg = NRF24_CMD_R_REGISTER | reg;

	NRF24_CSN_LOW;
	nRF24_SendSpi(&reg, 1);
	nRF24_ReadSpi(&result, 1);
	NRF24_CSN_HIGH;

	return result;
}

static void nRF24_ReadRegisters(uint8_t reg, uint8_t* ret, uint8_t len)
{
	reg = NRF24_CMD_R_REGISTER | reg;

	NRF24_CSN_LOW;

	nRF24_SendSpi(&reg, 1);
	nRF24_ReadSpi(ret, len);//todo

	NRF24_CSN_HIGH;
}

static void nRF24_ReadRegisters_IT(uint8_t reg, uint8_t* ret, uint8_t len)
{
	reg = NRF24_CMD_R_REGISTER | reg;

	NRF24_CSN_LOW;

	nRF24_SendSpi(&reg, 1);
	nRF24_ReadSpi_IT(ret, len);
}

static void nRF24_WriteRegister(uint8_t reg, uint8_t val)
{
	uint8_t tmp[2];

	tmp[0] = NRF24_CMD_W_REGISTER | reg;
	tmp[1] = val;

	NRF24_CSN_LOW;

	nRF24_SendSpi(tmp, 2);

	NRF24_CSN_HIGH;
}

static void nRF24_WriteRegisters(uint8_t reg, uint8_t* val, uint8_t len)
{
	reg = NRF24_CMD_W_REGISTER | reg;

	NRF24_CSN_LOW;

	nRF24_SendSpi(&reg, 1);
	nRF24_SendSpi(val, len);

	NRF24_CSN_HIGH;
}

static void nRF24_WriteRegisters_IT(uint8_t reg, uint8_t* val, uint8_t len)
{
	reg = NRF24_CMD_W_REGISTER | reg;

	NRF24_CSN_LOW;

	nRF24_SendSpi(&reg, 1);
	nRF24_SendSpi_IT(val, len);

	NRF24_Tx_Inte = 1;
	//NRF24_CSN_HIGH;
}



void nRF24_RX_Mode(void)
{
	uint8_t config = nRF24_ReadConfig();
	// Restore pipe 0 adress after comeback from TX mode
	nRF24_SetRXAddress(0, addr_p0_backup);
	// PWR_UP bit set
	config |= (1<<NRF24_PWR_UP);
	// PRIM_RX bit set
	config |= (1<<NRF24_PRIM_RX);
	nRF24_WriteConfig(config);
	// Reset status
	nRF24_WriteStatus((1<<NRF24_RX_DR)|(1<<NRF24_TX_DS)|(1<<NRF24_MAX_RT));
	// Flush RX
	nRF24_FlushRX();
	// Flush TX
	nRF24_FlushTX();

	NRF24_CE_HIGH;
	//nRF24_Delay(1);
}

void nRF24_TX_Mode(void)
{
	NRF24_CE_LOW;

	uint8_t config = nRF24_ReadConfig();
	// PWR_UP bit set
	config |= (1<<NRF24_PWR_UP);
	// PRIM_RX bit low
	config &= ~(1<<NRF24_PRIM_RX);
	nRF24_WriteConfig(config);
	// Reset status
	nRF24_WriteStatus((1<<NRF24_RX_DR)|(1<<NRF24_TX_DS)|(1<<NRF24_MAX_RT));
	// Flush RX
	nRF24_FlushRX();
	// Flush TX
	nRF24_FlushTX();
	//important wait 2ms for next interactionw with NRF24
	//nRF24_Delay(1);
}



uint8_t nRF24_ReadConfig(void)
{
	return (nRF24_ReadRegister(NRF24_CONFIG));
}

void nRF24_WriteConfig(uint8_t conf)
{
	nRF24_WriteRegister(NRF24_CONFIG, conf);
}

void nRF24_SetPALevel(uint8_t lev)
{
	uint8_t rf_setup = nRF24_ReadRegister(NRF24_RF_SETUP);
	rf_setup &= 0xF8; // Clear PWR bits
	rf_setup |= (lev<<1);
	nRF24_WriteRegister(NRF24_RF_SETUP, rf_setup);
}

void nRF24_SetDataRate(uint8_t dr)
{
	uint8_t rf_setup = nRF24_ReadRegister(NRF24_RF_SETUP);
	rf_setup &= 0xD7; // Clear DR bits (1MBPS)
	if(dr == NRF24_RF_DR_250KBPS)
		rf_setup |= (1<<NRF24_RF_DR_LOW);
	else if(dr == NRF24_RF_DR_2MBPS)
		rf_setup |= (1<<NRF24_RF_DR_HIGH);
	nRF24_WriteRegister(NRF24_RF_SETUP, rf_setup);
}

uint8_t nRF24_ReadStatus(void)
{
	return (nRF24_ReadRegister(NRF24_STATUS));
}

void nRF24_WriteStatus(uint8_t st)
{
	nRF24_WriteRegister(NRF24_STATUS, st);
}

void nRF24_FlushRX(void)
{
	uint8_t command = NRF24_CMD_FLUSH_RX;

	NRF24_CSN_LOW;
	nRF24_SendSpi(&command, 1);
	NRF24_CSN_HIGH;
}

void nRF24_FlushTX(void)
{
	uint8_t command = NRF24_CMD_FLUSH_TX;

	NRF24_CSN_LOW;
	nRF24_SendSpi(&command, 1);
	NRF24_CSN_HIGH;
}

void nRF24_EnableCRC(uint8_t onoff)
{
	uint8_t config = nRF24_ReadConfig();

	if(onoff)
		config |= (1<<NRF24_EN_CRC);
	else
		config &= ~(1<<NRF24_EN_CRC);
	nRF24_WriteConfig(config);
}

void nRF24_SetCRCLength(uint8_t crcl)
{
	uint8_t config = nRF24_ReadConfig();
	if(crcl == NRF24_CRC_WIDTH_2B)
		config |= (1<<NRF24_CRCO);
	else
		config &= ~(1<<NRF24_CRCO);
	nRF24_WriteConfig(config);
}

void nRF24_SetRetries(uint8_t ard, uint8_t arc)
{
	// ard * 250us, arc repeats
	nRF24_WriteRegister(NRF24_SETUP_RETR, (((ard & 0x0F)<<NRF24_ARD) | ((arc & 0x0F)<<NRF24_ARC)));
}

void nRF24_SetRFChannel(uint8_t channel)
{
	nRF24_WriteRegister(NRF24_RF_CH, (channel & 0x7F));
}

void nRF24_SetPayloadSize(uint8_t pipe, uint8_t size)
{
	if(pipe > 5)
		pipe = 5; // Block too high pipe number
	nRF24_WriteRegister(NRF24_RX_PW_P0 + pipe , (size & 0x3F));
}

void nRF24_EnablePipe(uint8_t pipe, uint8_t onoff)
{
	if(pipe > 5)
		pipe = 5; // Block too high pipe number
	uint8_t enable_pipe = nRF24_ReadRegister(NRF24_EN_RXADDR);
	if(onoff == 1)
		enable_pipe |= (1<<pipe);
	else
		enable_pipe &= ~(1<<pipe);
	nRF24_WriteRegister(NRF24_EN_RXADDR, enable_pipe);
}

void nRF24_AutoACK(uint8_t pipe, uint8_t onoff)
{
	if(pipe > 5)
		pipe = 5; // Block too high pipe number
	uint8_t enaa = nRF24_ReadRegister(NRF24_EN_AA);
	if(onoff == 1)
		enaa |= (1<<pipe);
	else
		enaa &= ~(1<<pipe);
	nRF24_WriteRegister(NRF24_EN_AA, enaa);
}

void nRF24_SetAddressWidth(uint8_t size)
{
	if(size > 5)
		size = 5; // Maximum are 5 bytes
	if(size < 3)
		size = 3; // Minimum are 3 bytes
	nRF24_WriteRegister(NRF24_SETUP_AW, ((size-2) & 0x03));
}

void nRF24_SetRXAddress(uint8_t pipe, uint8_t* address)
{
	// pipe 0 and pipe 1 are fully 40-bits storaged
	// pipe 2-5 is storaged only with last byte. Rest are as same as pipe 1
	// pipe 0 and 1 are LSByte first so they are needed to reverse address
	if((pipe == 0) || (pipe == 1))
	{
		uint8_t i;
		uint8_t address_rev[NRF24_ADDR_SIZE];
		for(i = 0; i<NRF24_ADDR_SIZE; i++)
			address_rev[NRF24_ADDR_SIZE - 1 - i] = address[i];
		nRF24_WriteRegisters(NRF24_RX_ADDR_P0 + pipe, address_rev, NRF24_ADDR_SIZE);
	}
	else
		nRF24_WriteRegister(NRF24_RX_ADDR_P0 + pipe, address[NRF24_ADDR_SIZE-1]);
}

void nRF24_SetTXAddress(uint8_t* address)
{
	// TX address is storaged similar to RX pipe 0 - LSByte first
	uint8_t i;
	uint8_t address_rev[NRF24_ADDR_SIZE];

	nRF24_ReadRegisters(NRF24_RX_ADDR_P0, address_rev, NRF24_ADDR_SIZE); // Backup P0 address
	for(i = 0; i<NRF24_ADDR_SIZE; i++)
		addr_p0_backup[NRF24_ADDR_SIZE - 1 - i] = address_rev[i]; //Reverse P0 address

	for(i = 0; i<NRF24_ADDR_SIZE; i++)
		address_rev[NRF24_ADDR_SIZE - 1 - i] = address[i];
	//make pipe 0 address backup;

	nRF24_WriteRegisters(NRF24_RX_ADDR_P0, address_rev, NRF24_ADDR_SIZE); // Pipe 0 must be same for auto ACk
	nRF24_WriteRegisters(NRF24_TX_ADDR, address_rev, NRF24_ADDR_SIZE);

}

void nRF24_ClearInterrupts(void)
{
	uint8_t status = nRF24_ReadStatus();
	status |= (7<<4); // Clear bits 4, 5, 6.
	nRF24_WriteStatus(status);
}

void nRF24_EnableRXDataReadyIRQ(uint8_t onoff)
{
	uint8_t config = nRF24_ReadConfig();

	if(!onoff)
		config |= (1<<NRF24_RX_DR);
	else
		config &= ~(1<<NRF24_RX_DR);

	nRF24_WriteConfig(config);
}

void nRF24_EnableTXDataSentIRQ(uint8_t onoff)
{
	uint8_t config = nRF24_ReadConfig();

	if(!onoff)
		config |= (1<<NRF24_TX_DS);
	else
		config &= ~(1<<NRF24_TX_DS);

	nRF24_WriteConfig(config);
}

void nRF24_EnableMaxRetransmitIRQ(uint8_t onoff)
{
	uint8_t config = nRF24_ReadConfig();

	if(!onoff)
		config |= (1<<NRF24_MAX_RT);
	else
		config &= ~(1<<NRF24_MAX_RT);

	nRF24_WriteConfig(config);
}

void nRF24_WriteTXPayload(uint8_t * data/*, uint8_t size*/)
{
	nRF24_WriteRegisters(NRF24_CMD_W_TX_PAYLOAD, data, NRF24_PAYLOAD_SIZE);
	//nRF24_WaitTX();
}

void nRF24_WriteTXPayload_IT(uint8_t * data)
{
	nRF24_WriteRegisters_IT(NRF24_CMD_W_TX_PAYLOAD, data, NRF24_PAYLOAD_SIZE);
}

void nRF24_WaitTX()
{
	uint8_t status;
	NRF24_CE_HIGH;
	nRF24_Delay(1);
	NRF24_CE_LOW;
	do
	{
		nRF24_Delay(1);
		status = nRF24_ReadStatus();
	}while(!((status & (1<<NRF24_MAX_RT)) || (status & (1<<NRF24_TX_DS))));

}

void nRF24_ReadRXPaylaod(uint8_t *data/*, uint8_t *size*/)
{
	nRF24_ReadRegisters(NRF24_CMD_R_RX_PAYLOAD, data, NRF24_PAYLOAD_SIZE);
	nRF24_WriteRegister(NRF24_STATUS, (1<<NRF24_RX_DR));

	if(nRF24_ReadStatus() & (1<<NRF24_TX_DS)){
		nRF24_WriteRegister(NRF24_STATUS, (1<<NRF24_TX_DS));
	}
//	*size = nRF24_GetDynamicPayloadSize();
//	nRF24_ReadRegisters(NRF24_CMD_R_RX_PAYLOAD, data, *size);
//
//	nRF24_WriteRegister(NRF24_STATUS, (1<NRF24_RX_DR));
//	if(nRF24_ReadStatus() & (1<<NRF24_TX_DS))
//		nRF24_WriteRegister(NRF24_STATUS, (1<<NRF24_TX_DS));
}

void nRF24_ReadRXPaylaod_IT(uint8_t *data){
	nRF24_ReadRegisters_IT(NRF24_CMD_R_RX_PAYLOAD, data, NRF24_PAYLOAD_SIZE);
	nRF24_WriteRegister(NRF24_STATUS, (1<<NRF24_RX_DR));

	if(nRF24_ReadStatus() & (1<<NRF24_TX_DS)){
		nRF24_WriteRegister(NRF24_STATUS, (1<<NRF24_TX_DS));
	}
}

uint8_t nRF24_GetDynamicPayloadSize(void)
{
    uint8_t result = 0;

    result = nRF24_ReadRegister(NRF24_CMD_R_RX_PL_WID);

    if (result > 32) // Something went wrong :)
    {
        nRF24_FlushRX();
        HAL_Delay(2);
        return 0;
    }
    return result;
}



uint8_t nRF24_RXAvailible(void)
{
	uint8_t status = nRF24_ReadStatus();

	// RX FIFO Interrupt
	if ((status & (1 << 6)))
	{
		nrf24_rx_flag = 1;
		status |= (1<<6); // Interrupt flag clear
		nRF24_WriteStatus(status);
		return 1;
	}
	return 0;
}

uint8_t nRF24_IsRxEmpty(void)
{
	uint8_t FifoStatus;

	FifoStatus = nRF24_ReadFifoStatus();

	if(FifoStatus & (1<<NRF24_RX_EMPTY))
	{
		return 1;
	}

	return 0;
}

uint8_t nRF24_IsBitSetInFifoStatus(uint8_t Bit)
{
	uint8_t FifoStatus;

	FifoStatus = nRF24_ReadFifoStatus();

	if(FifoStatus & (1<<Bit))
	{
		return 1;
	}

	return 0;
}

uint8_t nRF24_ReadFifoStatus(void)
{
	return (nRF24_ReadRegister(NRF24_FIFO_STATUS));
}



void nRF24_Init(SPI_HandleTypeDef *hspi)
{
	hspi_nrf = hspi;

	NRF24_CE_LOW;
	NRF24_CSN_HIGH;

	HAL_Delay(10); // Wait for radio power up

	nRF24_SetPALevel(NRF24_PA_PWR_0dBM); // Radio power
	nRF24_SetDataRate(NRF24_RF_DR_2MBPS); // Data Rate
	nRF24_EnableCRC(1); // Enable CRC
	nRF24_SetCRCLength(NRF24_CRC_WIDTH_1B); // CRC Length 1 byte
	nRF24_SetRetries(0x00, 0x00); // 1000us, 0 times


#if (NRF24_DYNAMIC_PAYLOAD == 1)
	nRF24_WriteRegister(NRF24_FEATURE, nRF24_ReadRegister(NRF24_FEATURE) | (1<<NRF24_EN_DPL)); // Enable dynamic payload feature
	nRF24_WriteRegister(NRF24_DYNPD, 0x3F); // Enable dynamic payloads for all pipes
#else
	nRF24_WriteRegister(NRF24_DYNPD, 0); // Disable dynamic payloads for all pipes
	nRF24_SetPayloadSize(0, NRF24_PAYLOAD_SIZE); // Set 32 bytes payload for pipe 0
#endif
	nRF24_SetRFChannel(15); // Set RF channel for transmission
	nRF24_EnablePipe(0, 1); // Enable pipe 0
	nRF24_AutoACK(0, 1); // Enable auto ACK for pipe 0
	nRF24_SetAddressWidth(NRF24_ADDR_SIZE); // Set address size

	HAL_Delay(1);

	nRF24_EnableRXDataReadyIRQ(1);
	nRF24_EnableTXDataSentIRQ(0);
	nRF24_EnableMaxRetransmitIRQ(0);

	HAL_Delay(1);

	nRF24_ClearInterrupts();


}
void nRF24_Inittest(void)
{
//	xz[0] = nRF24_ReadRegister(NRF24_RF_SETUP); // Radio power
//	//z = nRF24_ReadRegister(NRF24_RF_SETUP); // Data Rate
//	xz[1] = nRF24_ReadConfig(); // Enable CRC //
//	//z = nRF24_SetCRCLength(NRF24_CRC_WIDTH_1B); // CRC Length 1 byte
//	xz[2] = nRF24_ReadRegister(NRF24_SETUP_RETR); // 1000us, 7 times
//	xz[3] = nRF24_ReadRegister(NRF24_RF_CH);
//
//#if (NRF24_DYNAMIC_PAYLOAD == 1)
//	z = nRF24_ReadRegister(NRF24_FEATURE); // Enable dynamic payload feature
//	z = nRF24_ReadRegister(NRF24_DYNPD); // Enable dynamic payloads for all pipes
//#else
//	//nRF24_WriteRegister(NRF24_DYNPD, 0); // Disable dynamic payloads for all pipes
//	//nRF24_SetPayloadSize(0, NRF24_PAYLOAD_SIZE); // Set 32 bytes payload for pipe 0
//	xz[4] = nRF24_ReadRegister(NRF24_RX_PW_P0);
//#endif
//	xz[5] = nRF24_ReadRegister(NRF24_EN_RXADDR);
//	xz[6] = nRF24_ReadRegister(NRF24_EN_AA);
//	xz[7] = nRF24_ReadRegister(NRF24_SETUP_AW);
//	xz[8] = nRF24_ReadConfig();

}
