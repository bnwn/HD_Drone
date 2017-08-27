#include <stdio.h>
#include "PN020Series.h"
#include "timer_delay.h"
#include "rf.h"
#include "common.h"

//const uint8_t TX_ADDRESS_DEF[5] = {0xCC,0x6C,0x47,0x90,0x53};    		//RF 地址：接收端和发送端需一致 \u6c47\u9053"汇道"
const uint8_t TX_ADDRESS_DEF[5] = {0xCC,0xCC,0xCC,0xCC,0xCC};    		//RF 地址：接收端和发送端需一致 \u6c47\u9053"汇道"
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
    SPI_WriteReg(W_REGISTER + CONFIG,  0X8E);							// 将RF设置成TX模式
    delay_ms(1);
}


/******************************************************************************/
//            RF_RxMode
//            将RF设置成RX模式，准备接收数据
/******************************************************************************/
void RF_RxMode(void)
{
    CE_LOW;
    SPI_WriteReg(W_REGISTER + CONFIG,  0X8F );							// 将RF设置成RX模式
    CE_HIGH;											// Set CE pin high 开始接收数据
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
    return SPI_ReadReg(RF_STATUS)&0x70;								//读取RF的状态 
}
/******************************************************************************/
//            ucRF_GetRSSI
//                读取rssi 值
/******************************************************************************/
uint8_t ucRF_GetRSSI(void)
{
    return (SPI_ReadReg(DATAOUT));								//读取RF RSSI
}
/******************************************************************************/
//            RF_ClearStatus
//                clear RF IRQ
/******************************************************************************/
void RF_ClearStatus(void)
{
    SPI_WriteReg(W_REGISTER + RF_STATUS,0x70);							//清除RF的IRQ标志 
}

/******************************************************************************/
//            RF_ClearFIFO
//                clear RF TX/RX FIFO
/******************************************************************************/
void RF_ClearFIFO(void)
{
    SPI_WriteReg(FLUSH_TX, 0);			                                		//清除RF 的 TX FIFO		
    SPI_WriteReg(FLUSH_RX, 0);                                                   		//清除RF 的 RX FIFO	
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
//            读出接收到的数据：
//            参数：
//              1. ucPayload：存储读取到的数据的Buffer
//              2. length:    读取的数据长度
//              Return:
//              1. 0: 没有接收到数据
//              2. 1: 读取接收到的数据成功
//              note: Only use in Rx Mode
//              length 通常等于 PAYLOAD_WIDTH
/******************************************************************************/
uint8_t ucRF_DumpRxData( uint8_t *ucPayload,  uint8_t length)
{ 
   if(ucRF_GetStatus()&RX_DR_FLAG)
   {
   
        CE_LOW;
        SPI_ReadBuf(R_RX_PAYLOAD, ucPayload, length);                                		//将接收到的数据读出到ucPayload，且清除rxfifo
        RF_ClearFIFO();
        RF_ClearStatus ();                              		                        //清除Status     
        CE_HIGH;                                                                    		//继续开始接        
        return 1;
    }
     return 0;
}




/******************************************************************************/
//  发送数据 
//            参数：
//              1. ucTXPayload：需要发送的数据首地址
//              2. length:  需要发送的数据长度
/******************************************************************************/

void RF_Tx_TransmintData( uint8_t *ucTXPayload,  uint8_t length)
{


    
    if(!ucRF_GetStatus())                                                                               // rf 处于空闲状态才发送数据
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
//发送结果
//            参数：只在增强型模式下，使能ack带payload有效
//              1. ucAckPayload：ackpayload的首地址
//              2. length:ackpayload 的长度

/*****************************************************************************/
void  RF_Tx_CheckResult(uint8_t *ucAckPayload,  uint8_t length)
{

    
        switch(ucRF_GetStatus())
        {
          
          case	RX_TX_FLAG:		                                                        //增强型发送成功且收到payload          
                SPI_ReadBuf(R_RX_PAYLOAD,ucAckPayload, length);           

         case	TX_DS_FLAG: 		                                                        // 普通型发送完成 或 增强型发送成功	
                Payload_Count++;                                                                // 增强型模式，累计ack次数                                                                                
         case	MAX_RT_FLAG:		                                                        // 增强型发送超时失败	                                          
                RF_ClearFIFO();
                RF_ClearStatus ();
                break;
         default:			
                break;			
          }

      
}




////////////////////////////////////////////////////////////////////////////////

//          以下部分与RF通信相关，不建议修改
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
//            		进入载波模式
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
    SPI_WriteReg(W_REGISTER + RF_CH, ucChannel_Set);						//单载波频点	   
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



