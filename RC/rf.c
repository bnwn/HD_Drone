#include <stdio.h>
#include "PN020Series.h"
#include "timer_delay.h"
#include "rf.h"
#include "common.h"

//const uint8_t TX_ADDRESS_DEF[5] = {0xCC,0x6C,0x47,0x90,0x53};    		//RF ��ַ�����ն˺ͷ��Ͷ���һ�� \u6c47\u9053"���"
const uint8_t TX_ADDRESS_DEF[5] = {0xCC,0xCC,0xCC,0xCC,0xCC};    		//RF ��ַ�����ն˺ͷ��Ͷ���һ�� \u6c47\u9053"���"
unsigned short   Payload_Count = 0;
//typedef union 
//{
//    uint8_t ucPayload[PAYLOAD_WIDTH];

//}RF_PAYLOAD;
//RF_PAYLOAD  RF_Payload;
//uint8_t TxPayloadLength = PAYLOAD_WIDTH;
//uint8_t AckPayloadLength = 0x00;


/******************************************************************************/
//            RF_TxMode
//                Set RF into TX mode
/******************************************************************************/
void RF_TxMode(void)
{
    CE_LOW;
    SPI_WriteReg(W_REGISTER + CONFIG,  0X8E);							// ��RF���ó�TXģʽ
    delay_ms(1);
}


/******************************************************************************/
//            RF_RxMode
//            ��RF���ó�RXģʽ��׼����������
/******************************************************************************/
void RF_RxMode(void)
{
    CE_LOW;
    SPI_WriteReg(W_REGISTER + CONFIG,  0X8F );							// ��RF���ó�RXģʽ
    CE_HIGH;											// Set CE pin high ��ʼ��������
   // delay_ms(2);
   // Delay(9000);
    delay_ms(2);
}

/******************************************************************************/
//            RF_GetStatus
//            read RF IRQ status,3bits return
/******************************************************************************/
uint8_t ucRF_GetStatus(void)
{
    return SPI_ReadReg(RF_STATUS)&0x70;								//��ȡRF��״̬ 
}
/******************************************************************************/
//            ucRF_GetRSSI
//                ��ȡrssi ֵ
/******************************************************************************/
uint8_t ucRF_GetRSSI(void)
{
    return (SPI_ReadReg(DATAOUT));								//��ȡRF RSSI
}
/******************************************************************************/
//            RF_ClearStatus
//                clear RF IRQ
/******************************************************************************/
void RF_ClearStatus(void)
{
    SPI_WriteReg(W_REGISTER + RF_STATUS,0x70);							//���RF��IRQ��־ 
}

/******************************************************************************/
//            RF_ClearFIFO
//                clear RF TX/RX FIFO
/******************************************************************************/
void RF_ClearFIFO(void)
{
    SPI_WriteReg(FLUSH_TX, 0);			                                		//���RF �� TX FIFO		
    SPI_WriteReg(FLUSH_RX, 0);                                                   		//���RF �� RX FIFO	
}

/******************************************************************************/
//            RF_SetChannel
//                Set RF TX/RX channel:Channel
/******************************************************************************/
void RF_SetChannel( uint8_t Channel)
{    
    CE_LOW;
    SPI_WriteReg(W_REGISTER + RF_CH, Channel);
}


/******************************************************************************/
//            ucRF_DumpRxData
//            �������յ������ݣ�
//            ������
//              1. ucPayload���洢��ȡ�������ݵ�Buffer
//              2. length:    ��ȡ�����ݳ���
//              Return:
//              1. 0: û�н��յ�����
//              2. 1: ��ȡ���յ������ݳɹ�
//              note: Only use in Rx Mode
//              length ͨ������ PAYLOAD_WIDTH
/******************************************************************************/
uint8_t ucRF_DumpRxData( uint8_t *ucPayload,  uint8_t length)
{ 
   if(ucRF_GetStatus()&RX_DR_FLAG)
   {
   
        CE_LOW;
        SPI_ReadBuf(R_RX_PAYLOAD, ucPayload, length);                                		//�����յ������ݶ�����ucPayload�������rxfifo
        RF_ClearFIFO();
        RF_ClearStatus ();                              		                        //���Status     
        CE_HIGH;                                                                    		//������ʼ��        
        return 1;
    }
     return 0;
}




/******************************************************************************/
//  �������� 
//            ������
//              1. ucTXPayload����Ҫ���͵������׵�ַ
//              2. length:  ��Ҫ���͵����ݳ���
/******************************************************************************/

void RF_Tx_TransmintData( uint8_t *ucTXPayload,  uint8_t length)
{


    
    if(!ucRF_GetStatus())                                                                               // rf ���ڿ���״̬�ŷ�������
    {       
            SPI_WriteBuf(W_TX_PAYLOAD, ucTXPayload, length);                               		//write data to txfifo        
            CE_HIGH;                                                                    		//rf entery tx mode start send data 
           // delay_10us(60);                                                              		//keep ce high at least 150us
           // Delay(2000);
            delay_ms(1);
            CE_LOW;
    }
}

/******************************************************************************/
//���ͽ��
//            ������ֻ����ǿ��ģʽ�£�ʹ��ack��payload��Ч
//              1. ucAckPayload��ackpayload���׵�ַ
//              2. length:ackpayload �ĳ���

/*****************************************************************************/
void  RF_Tx_CheckResult(uint8_t *ucAckPayload,  uint8_t length)
{

    
        switch(ucRF_GetStatus())
        {
          
          case	RX_TX_FLAG:		                                                        //��ǿ�ͷ��ͳɹ����յ�payload          
                SPI_ReadBuf(R_RX_PAYLOAD,ucAckPayload, length);           

         case	TX_DS_FLAG: 		                                                        // ��ͨ�ͷ������ �� ��ǿ�ͷ��ͳɹ�	
                Payload_Count++;                                                                // ��ǿ��ģʽ���ۼ�ack����                                                                                
         case	MAX_RT_FLAG:		                                                        // ��ǿ�ͷ��ͳ�ʱʧ��	                                          
                RF_ClearFIFO();
                RF_ClearStatus ();
                break;
         default:			
                break;			
          }

      
}




////////////////////////////////////////////////////////////////////////////////

//          ���²�����RFͨ����أ��������޸�
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
//            PN006_Initial
//                Initial RF
/******************************************************************************/
void RF_Init(void)
{
    uint8_t  BB_cal_data[]    = {0x0A,0x6D,0x67,0x9C,0x46}; 
    uint8_t  RF_cal_data[]    = {0xF6,0x37,0x5D};
    uint8_t  RF_cal2_data[]   = {0x45,0x21,0xef,0xAC,0x5A,0x50};
    uint8_t  Dem_cal_data[]   = {0x01};  
    uint8_t  Dem_cal2_data[]  = {0x0b,0xDF,0x02};   
  

    uint8_t feature=0x00;

        feature |=0x20;

if(PAYLOAD_WIDTH >32)
        feature |= 0x18;  

                    
    SPI_WriteReg(FLUSH_TX, 0);									// CLEAR TXFIFO		    			 
    SPI_WriteReg(FLUSH_RX, 0);									// CLEAR  RXFIFO
    SPI_WriteReg(W_REGISTER + RF_STATUS, 0x70);							// CLEAR  STATUS	
    SPI_WriteReg(W_REGISTER + EN_RXADDR, 0x01);							// Enable Pipe0
    SPI_WriteReg(W_REGISTER + RF_CH,     DEFAULT_CHANNEL);                                       // 2478M HZ
    SPI_WriteReg(W_REGISTER + RX_PW_P0,  PAYLOAD_WIDTH);						// 8 bytes
    SPI_WriteBuf(W_REGISTER + TX_ADDR,   ( uint8_t*)TX_ADDRESS_DEF, sizeof(TX_ADDRESS_DEF));	// Writes TX_Address to PN006
    SPI_WriteBuf(W_REGISTER + RX_ADDR_P0,( uint8_t*)TX_ADDRESS_DEF, sizeof(TX_ADDRESS_DEF));	// RX_Addr0 same as TX_Adr for Auto.Ack   
    SPI_WriteBuf(W_REGISTER + BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
    SPI_WriteBuf(W_REGISTER + RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
    SPI_WriteBuf(W_REGISTER + DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
    SPI_WriteBuf(W_REGISTER + RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
    SPI_WriteBuf(W_REGISTER + DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
					

    SPI_WriteReg(W_REGISTER + RF_SETUP,  RF_POWER);						// 13DBM  		
  
    
#if(TRANSMIT_TYPE == TRANS_ENHANCE_MODE)      
    SPI_WriteReg(W_REGISTER + SETUP_RETR,0x03);							//  3 retrans... 	
    SPI_WriteReg(W_REGISTER + EN_AA,     0x01);							// Enable Auto.Ack:Pipe0  	
    SPI_WriteReg(ACTIVATE, 0x73);  

    #if(1== EN_DYNPLOAD)                                                    
        feature |= 0x04;                                                                                //  ENDYNPD                
        SPI_WriteReg(W_REGISTER +DYNPD, 0x01);     
    #endif  
    #if(1==EN_ACK_PAYLOAD)                                                                             // en ack+payload
         feature |= 0x02;
    #endif    

    
#elif(TRANSMIT_TYPE == TRANS_BURST_MODE)                                                                
    SPI_WriteReg(W_REGISTER + SETUP_RETR,0x00);							// Disable retrans... 	
    SPI_WriteReg(W_REGISTER + EN_AA,     0x00);							// Disable AutoAck 
#endif
        SPI_WriteReg(W_REGISTER +FEATURE, feature);   
    CE_LOW;


}


/******************************************************************************/
//            		�����ز�ģʽ
/******************************************************************************/
void RF_Carrier( uint8_t ucChannel_Set)
{
    uint8_t BB_cal_data[]    = {0x0A,0x6D,0x67,0x9C,0x46}; 
    uint8_t RF_cal_data[]    = {0xF6,0x37,0x5D};
    uint8_t RF_cal2_data[]   = {0x45,0x21,0xEF,0xAC,0x5A,0x50};
    uint8_t Dem_cal_data[]   = {0xE1}; 								
    uint8_t Dem_cal2_data[]  = {0x0B,0xDF,0x02};  

    SPI_WriteReg(RST_FSPI, 0x5A);								//Software Reset    			
    SPI_WriteReg(RST_FSPI, 0XA5);
    SPI_WriteReg(W_REGISTER + FEATURE, 0x20);
    CE_LOW;
    //delay_ms(200);
    delay_ms(200);
   //  Delay(9000);
    SPI_WriteReg(W_REGISTER + RF_CH, ucChannel_Set);						//���ز�Ƶ��	   
    SPI_WriteReg(W_REGISTER + RF_SETUP, RF_POWER);      						//13dbm
    SPI_WriteBuf(W_REGISTER + BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
    SPI_WriteBuf(W_REGISTER + RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
    SPI_WriteBuf(W_REGISTER + DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
    SPI_WriteBuf(W_REGISTER + RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
    SPI_WriteBuf(W_REGISTER + DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
}



/******************************************************************************/
//            RF_PacketData
//                ready for tx data
/******************************************************************************/
//void RFAPI_PacketData(void)
//{
//     uint8_t i; 
//    ++RF_Payload.ucPayload[0];
//    
//    for(i=1;i<sizeof(RF_Payload.ucPayload);i++)
//    {
//        RF_Payload.ucPayload[i] = 0xC0 | i;
//    }
//}


// void  RF_Test(void)
//{
//    int i = 1000;
//    RF_Init();
//    RF_TxMode();
//    printf("\n  rf init sucess \n");
//    printf("\n  rf start transmit 1000 packets paylaod ...\n");
//    while(i--)
//    {
//      
//     delay_ms(10);        
//        {
//            RFAPI_PacketData();                                                        //prepare data
//            RF_Tx_CheckResult(RF_Payload.ucPayload,AckPayloadLength);   
//            RF_Tx_TransmintData(RF_Payload.ucPayload,TxPayloadLength);           
//        }
//    }
//}




/***************************************end of file ************************************/



