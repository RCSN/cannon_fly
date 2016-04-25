#include "stm32f4xx_cannon_v2.h"
#include "NRF24L01.h"
extern SPI_HandleTypeDef hnucleo_Spi2;
uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01���յ�������
uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];//nrf24l01��Ҫ���͵�����


uint8_t TX_ADDRESS[5]={0x00,0x00,0x00,0x00,0xA3};
//�޸ĸý��պͷ��͵�ַ�����Թ������������ͬһ������У����ݲ��ܸ���
uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0x00,0x00,0x00,0x00,0xA1};	//���յ�ַ				


//д�Ĵ���
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    SPI2_CSN_L();		
    HAL_Delay(1);	
    status = HAL_SPI_Transmit(&hnucleo_Spi2,&reg, 1, 100);  
	  HAL_SPI_Transmit(&hnucleo_Spi2,&value, 1, 100);
  //  SPI2_RW(value);		  /* д���� */
	HAL_Delay(1);	
    SPI2_CSN_H();					  /* ��ֹ������ */
    return 	status;
}


//���Ĵ���
uint8_t NRF_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    SPI2_CSN_L();		
    HAL_Delay(1);		
    HAL_SPI_Transmit(&hnucleo_Spi2,   &reg, 1, 100); 
    HAL_SPI_Receive(&hnucleo_Spi2,  &reg_val, 1, 100);	  /* ��ȡ�üĴ����������� */
    SPI2_CSN_H();	
 
    return 	reg_val;
}


//д������
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    SPI2_CSN_L();				        /* ѡͨ���� */
    status = HAL_SPI_Transmit(&hnucleo_Spi2,&reg, 1, 100);	/* д�Ĵ�����ַ */
//    for(i=0; i<uchars; i++)
//    {
//        SPI2_RW(pBuf[i]);		/* д���� */
//    }
	  HAL_SPI_Transmit(&hnucleo_Spi2,pBuf,uchars, 100);
    SPI2_CSN_H();						/* ��ֹ������ */
    return 	status;	
}

//��������
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    SPI2_CSN_L();						/* ѡͨ���� */
	 HAL_Delay(1);
    status = HAL_SPI_Transmit(&hnucleo_Spi2, &reg, 1, 100); /* д�Ĵ�����ַ */
   // for(i=0; i<uchars; i++)
   // {
   //     pBuf[i] = SPI2_RW(0); /* ��ȡ�������� */ 	
   // }
	 HAL_SPI_Receive(&hnucleo_Spi2,pBuf,uchars, 100);
    SPI2_CSN_H();				 			/* ��ֹ������ */
    return 	status;
}

//д���ݰ�
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
    SPI2_CE_L();		 //StandBy Iģʽ	
    NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // װ������	
    SPI2_CE_H();		 //�ø�CE���������ݷ���
}

//��ʼ��
char NRF24L01_INIT(void)
{
	SPI2_Init();
	
	//check if NRF24L01 is in the SPI bus
	NRF24L01_Check();
	
	//set the NRF RX Address,priority to the latest address in eeprom
	//NRF24L01��Ƶ���յ�ַ������ƥ����eeprom����һ�ε�ַ
	//NRFmatching();								
}


//����ģʽ
void SetRX_Mode(void)
{
    SPI2_CE_L();
	  NRF_Write_Reg(FLUSH_RX,0xff);//���TX FIFO�Ĵ���			 
  	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
   	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,0);	     //����RFͨ��Ƶ��		  
  	NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);//����TX�������,0db����,2Mbps,���������濪��   
  	NRF_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
    SPI2_CE_H();
    //printf("NRF24L01 Set to Receiving Mode,RX_ADDR 0x%x...\r\n",RX_ADDRESS[4]);
} 


//��ѯ�ж�
void Nrf_Irq(void)
{
    uint8_t sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
    if(sta & (1<<RX_DR))//������ѵ��־λ
    {
        NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
    //    ReceiveDataFormNRF();    //�Լ����޸�
				NRF_Write_Reg(0x27, sta);//���nrf���жϱ�־λ
			  sta = 0;
    }
    
}


//���պ���
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;	
SPI2_CE_L();	
        //SPI2_SetSpeed(SPI_SPEED_4); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	sta=NRF_Read_Reg(NRFRegSTATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	
	if(sta&RX_OK)//���յ�����
	{
		NRF_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		//NRF_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		NRF_Write_Reg(NRF_WRITE_REG+NRFRegSTATUS,sta); //���TX_DS��MAX_RT�жϱ�־
		return 0; 
	}	   
	SPI2_CE_H();
	return 1;//û�յ��κ�����
}		



//�ж�SPI�ӿ��Ƿ����NRFоƬ�Ƿ����
uint8_t NRF24L01_Check(void) 
{ 
   uint8_t buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2}; 
   uint8_t buf1[5]; 
   uint8_t i=0; 
    
   /*д��5 ���ֽڵĵ�ַ.  */ 
   NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5); 
     
   /*����д��ĵ�ַ */ 
   NRF_Read_Buf(TX_ADDR,buf1,5); 
   
    /*�Ƚ�*/ 
   for (i=0;i<5;i++) 
   { 
      if (buf1[i]!=0xC2) 
      break; 
   } 
  
   if (i==5)  
   {BSP_LED_On(LED0);
	 return 1 ;}        //MCU ��NRF �ɹ����� 
   else      
  {BSP_LED_Off(LED0);
	return 0 ;}        //MCU��NRF����������    
} 




