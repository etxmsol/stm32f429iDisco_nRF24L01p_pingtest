
#include "nRF24l01P.h"
#include "main.h"

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

//Define the commands for operate the nRF24L01P
#define READ_nRF_REG    0x00  // Command for read register
#define WRITE_nRF_REG   0x20 	// Command for write register
#define RD_RX_PLOAD     0x61  // Command for read Rx payload
#define WR_TX_PLOAD     0xA0  // Command for write Tx payload
#define FLUSH_TX        0xE1 	// Command for flush Tx FIFO
#define FLUSH_RX        0xE2  // Command for flush Rx FIFO
#define REUSE_TX_PL     0xE3  // Command for reuse Tx payload
#define NOP             0xFF  // Reserve

//Define the register address for nRF24L01P
#define CONFIG          0x00  //  Configurate the status of transceiver, mode of CRC and the replay of transceiver status
#define EN_AA           0x01  //  Enable the atuo-ack in all channels
#define EN_RXADDR       0x02  //  Enable Rx Address
#define SETUP_AW        0x03  // Configurate the address width
#define SETUP_RETR      0x04  //  setup the retransmit
#define RF_CH           0x05  // Configurate the RF frequency
#define RF_SETUP        0x06  // Setup the rate of data, and transmit power
#define NRFRegSTATUS    0x07  //
#define OBSERVE_TX      0x08  //
#define CD              0x09  // Carrier detect
#define RX_ADDR_P0      0x0A  // Receive address of channel 0
#define RX_ADDR_P1      0x0B  // Receive address of channel 1
#define RX_ADDR_P2      0x0C  // Receive address of channel 2
#define RX_ADDR_P3      0x0D  // Receive address of channel 3
#define RX_ADDR_P4      0x0E  // Receive address of channel 4
#define RX_ADDR_P5      0x0F  // Receive address of channel 5
#define TX_ADDR         0x10  //       Transmit address
#define RX_PW_P0        0x11  //  Size of receive data in channel 0
#define RX_PW_P1        0x12  //  Size of receive data in channel 1
#define RX_PW_P2        0x13  //  Size of receive data in channel 2
#define RX_PW_P3        0x14  //  Size of receive data in channel 3
#define RX_PW_P4        0x15  // Size of receive data in channel 4
#define RX_PW_P5        0x16  //  Size of receive data in channel 5
#define FIFO_STATUS     0x17  // FIFO Status

#define _BV(x) (1<<(x))

#define nRF24l01_IRQ    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2); 
///**************************************************************************************

//define the private constants in this library
//#define TX_ADR_WIDTH 5
//#define RX_ADR_WIDTH 5

unsigned char TxBuf[Buffer_Size] = {0};
unsigned char RxBuf[Buffer_Size] = {0};


//define the initial Address
unsigned char  TX_ADDRESS[ADR_WIDTH]= {0xD0,0xD0,0xD0,0xD0,0xD0};
unsigned char  RX_ADDRESS[ADR_WIDTH]= {0xe1,0xF0,0xF0,0xF0,0xF0};
// unsigned char  TX_ADDRESS[ADR_WIDTH]= {0xB3,0xB4,0xB5,0xB6,0xF1};
// unsigned char  RX_ADDRESS[ADR_WIDTH]= {0xB3,0xB4,0xB5,0xB6,0xF1};
// unsigned char  TX_ADDRESS[ADR_WIDTH]= {0x01,0x23,0x45,0x67,0x89};
// unsigned char  RX_ADDRESS[ADR_WIDTH]= {0x01,0x23,0x45,0x67,0x89};
//Define the layer1:HW operation
unsigned char nRF24L01_SPI_Send_Byte(unsigned char dat);
void nRF24L01_SPI_NSS_L(void);
void nRF24L01_SPI_NSS_H(void);
void nRF24L01_CE_L(void);
void nRF24L01_CE_H(void);
//Define the layer2:Reg operation
unsigned char SPI_WR_Reg(unsigned char reg, unsigned char value);
unsigned char SPI_RD_Reg(unsigned char reg);
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len);
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len);


//Define the layer3:application operation
/****************************************

All the functions is in "nRF24l01P.h"

****************************************/

//Define the other function
void nRF24L01_Delay_us(unsigned long n);

//Define the layer3 functions



void RX_Mode(void)
{
  //nRF24L01_CE_L();

	nRF24L01_SPI_NSS_L();
	nRF24L01_SPI_Send_Byte(0xE1);           // flush Tx
	nRF24L01_SPI_NSS_H();                   //
	nRF24L01_SPI_NSS_L();                  	//
	nRF24L01_SPI_Send_Byte(0xE2);			// flush Rx
	nRF24L01_SPI_NSS_H();                   //

	SPI_WR_Reg(WRITE_nRF_REG + CONFIG, 0x0f & ~_BV(PWR_UP)); // disable power up

	SPI_WR_Reg(WRITE_nRF_REG + SETUP_AW, 0x03); // setup add width 5 bytes
	SPI_WR_Reg(WRITE_nRF_REG + RF_CH,10);// setup frequency
	SPI_WR_Reg(WRITE_nRF_REG + RF_SETUP,  0x07);// setup power and rate
	SPI_WR_Reg(WRITE_nRF_REG + RX_PW_P0,32); //Number of bytes in data P1
	SPI_WR_Reg(WRITE_nRF_REG + RX_PW_P1,32); //Number of bytes in data P1
	SPI_WR_Reg(WRITE_nRF_REG + EN_RXADDR, 0x03); //Enable data P1
	SPI_Write_Buf(WRITE_nRF_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH); // write address into tx_add
	SPI_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, TX_ADDRESS, ADR_WIDTH); // write address into rx_add_p0
	SPI_Write_Buf(WRITE_nRF_REG + RX_ADDR_P1, RX_ADDRESS, ADR_WIDTH); // write address into rx_add_p0
	SPI_WR_Reg(WRITE_nRF_REG + EN_AA, 0x3f);     //enable auto-ack for all pipes
	
	
    SPI_WR_Reg(WRITE_nRF_REG+NRFRegSTATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	SPI_WR_Reg(WRITE_nRF_REG + CONFIG, 0x0f); // enable power up and prx
  nRF24L01_CE_H();
  HAL_Delay(1);
}




void nRF24L01_TxPacket(unsigned char * tx_buf)
{
	nRF24L01_CE_L();
	SPI_WR_Reg(WRITE_nRF_REG + CONFIG, ( SPI_RD_Reg(CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX));

	HAL_Delay(1);		// 150us should suffice, but we only have ms delay in HAL (?)



	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);

	// start transmitting the payload
	nRF24L01_CE_H();

	uint8_t status;

	//uint32_t sent_at = __millis();
	const uint32_t timeout = 500; //ms to wait for timeout
	do
	{
		status = SPI_RD_Reg(NRFRegSTATUS);
	}
	while( ! ( status & ( _BV(TX_DS) | _BV(MAX_RT) ) ));

    SPI_WR_Reg(WRITE_nRF_REG+NRFRegSTATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

    // switch back to the default state: receiving
	nRF24L01_CE_L();
	SPI_WR_Reg(WRITE_nRF_REG + CONFIG, ( SPI_RD_Reg(CONFIG) | _BV(PWR_UP) ) | _BV(PRIM_RX));
    HAL_Delay(1);
	nRF24L01_CE_H();
}

unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
{
	unsigned char flag=0;
	unsigned char status;
	status=SPI_RD_Reg(NRFRegSTATUS);

	if(status & 0x40) //Data Ready RX FIFO interrupt
	{
		SPI_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);
		flag =1;
		SPI_WR_Reg(WRITE_nRF_REG+NRFRegSTATUS, 0x70); // Write 1 to clear bit
	}
	return flag;
}

//Define the layer2 functions
unsigned char SPI_RD_Reg(unsigned char reg)
{
	unsigned char reg_val;

	nRF24L01_SPI_NSS_L();                // CSN low, initialize SPI communication...
	nRF24L01_SPI_Send_Byte(reg);            // Select register to read from..
	reg_val = nRF24L01_SPI_Send_Byte(0);    // ..then read register value
	nRF24L01_SPI_NSS_H();                // CSN high, terminate SPI communication

	return(reg_val);        // return register value
}

unsigned char SPI_WR_Reg(unsigned char reg, unsigned char value)
{
	unsigned char status;

	nRF24L01_SPI_NSS_L();                  // CSN low, init SPI transaction
	status = nRF24L01_SPI_Send_Byte(reg);// select register
	nRF24L01_SPI_Send_Byte(value);             // ..and write value to it..
	nRF24L01_SPI_NSS_H();                   // CSN high again

	return(status);            // return nRF24L01 status unsigned char
}

unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len)
{
	unsigned int status,i;

	nRF24L01_SPI_NSS_L();                  // Set CSN low, init SPI tranaction
	status = nRF24L01_SPI_Send_Byte(reg);  // Select register to write to and read status unsigned char

  for(i=0;i<Len;i++)
  {
     pBuf[i] = nRF24L01_SPI_Send_Byte(0);
  }

	nRF24L01_SPI_NSS_H();

	return(status);                    // return nRF24L01 status unsigned char
}

unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len)
{
	unsigned int status,i;
	nRF24L01_SPI_NSS_L();
	status = nRF24L01_SPI_Send_Byte(reg);
	for(i=0; i<Len; i++) //
	{
		nRF24L01_SPI_Send_Byte(*pBuf);
		pBuf ++;
	}
	nRF24L01_SPI_NSS_H();    
	return(status);   
}

extern SPI_HandleTypeDef SpiHandle;


//Define the layer1 functions
unsigned char nRF24L01_SPI_Send_Byte(unsigned char dat)
{
	  /*##-2- Start the Full Duplex Communication process ########################*/
	  /* While the SPI in TransmitReceive process, user can transmit data through
	     "aTxBuffer" buffer & receive data through "aRxBuffer" */
	  /* Timeout is set to 5s */

	unsigned char result;
	  switch(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*)&dat, (uint8_t *)&result, 1, 5000))
	  {
	  case HAL_OK:
	    /* Communication is completed_____________________________________________*/
	    /* Compare the sent and received buffers */
		    /* Turn LED3 on: Transfer process is correct */
		    BSP_LED_On(LED3);
		  return result;


	  case HAL_TIMEOUT:
	    /* A Timeout occurred_____________________________________________________*/
	    /* Call Timeout Handler */
	    //Timeout_Error_Handler();
	    break;

	    /* An Error occurred______________________________________________________*/
	  case HAL_ERROR:
	    /* Call Timeout Handler */
	    //Error_Handler();
	    break;

	  default:
	    break;
	  }
	  return result;
}

void nRF24L01_SPI_NSS_H(void)
{
	HAL_GPIO_WritePin(GPIO_CS_CE, GPIO_Pin_CS, GPIO_PIN_SET);
}

void nRF24L01_SPI_NSS_L(void)
{
	HAL_GPIO_WritePin(GPIO_CS_CE, GPIO_Pin_CS, GPIO_PIN_RESET);
}

void nRF24L01_CE_L(void)
{
	HAL_GPIO_WritePin(GPIO_CS_CE, GPIO_Pin_CE, GPIO_PIN_RESET);
}

void nRF24L01_CE_H(void)
{
	HAL_GPIO_WritePin(GPIO_CS_CE, GPIO_Pin_CE, GPIO_PIN_SET);
}
