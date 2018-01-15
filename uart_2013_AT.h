#ifndef __UART_2013_AT_H__
#define __UART_2013_AT_H__

#include        <string.h>
#include        <stdlib.h>


extern void        LongToPanID( INT8U* pByte6, INT8U* pByte2 );
extern INT16U          cac_GetBroadDelayTime( INT16U tShiXi );
extern INT8U uart_DAUType( INT16U pNum, INT8U pType );
void DL2013_AT_function( INT8U* pBuf, INT16U pLen );
extern INT8U Get645FrameType( INT8U* p );
extern void rf_BroadcastTime( INT8U* pBuf, INT8U pLen );

#define ERROR_05H_F3_AT_TIME_OVER_FLOW  0x45
#define ERROR_05H_F100_AT_RSSIT_OVER_FLOW 0x51
#define ERROR_10H_F100_AT_SCALE_OVER_FLOW 0x52
#define ERROR_05H_F101_AT_TIME_OVER_FLOW  0x53
#define ERROR_05H_F4_AT_TIMEOUT_OVER_FLOW  0x54
#define ERROR_05H_F5_AT_SPEED_OVER_FLOW  0x55
#define ERROR_05H_F5_AT_POWER_OVER_FLOW  0x56
#define ERROR_04H_F1_AT_TIMEOUT_OVER_FLOW  0x57
#define ERROR_F0H_F210_AT_LEN_OVER_FLOW 0x58
#define ERROR_03H_F9_AT_PROTOCOL_TYPE_OVER_FLOW 0x59
#define ERROR_03H_F9_AT_PROTOCOL_LEN_OVER_FLOW 0x5a

#define ERROR_11H_F1_AT_ADD_NUM_OVER_FLOW 0x5b
#define ERROR_11H_F2_AT_DEL_NUM_OVER_FLOW 0x5c

#define ERROR_10H_F6_AT_NODE_OVER_FLOW 0x5d
#define ERROR_10H_F2_AT_NODE_OVER_FLOW 0x5e
#define ERROR_10H_F5_AT_NODE_OVER_FLOW 0x5f


#define ERROR_10H_F101_AT_NO_THIS_NODE 0x60



typedef struct
{
    unsigned int year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
} AT_time_t;


#define OFFSET_SECOND   946684800
#define SECOND_OF_DAY   86400



void NUM_2_ASCII_AT( INT32U num, INT8U* pBuf, INT8U* len );
void HEX_2_ASCII_AT( INT8U* pBufhex, INT8U* pBuf,  INT16U hex_len );
void AT_RTC_REQ( void );
    

INT8U IsLegal( int year, int mon, int day );


#endif