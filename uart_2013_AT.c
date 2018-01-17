#include    "prio_II.h"
#include        "uart_2013_AT.h"

extern  INT8U   uartTxBuf[UART_ARG_1_LEN];          // 定义串口发送缓冲区
extern  INT8U   uart_path[36];
extern  INT8U   tempBuf22[256];
extern  INT8U   backMeter;
extern INT16U TskOptimizeLimitTime;
extern INT8U        gFlgUrtCmpFile;                         // 档案同步流程标志
extern INT8U        FlagNew;                                                    // 上报标志位，用于激活主动注册

INT16U      xb_Capacity( INT16U pN );
INT8U   AT_Save_Last_Cmd_Buf[LEN_RF], AT_Save_Last_Cmd_Buf_Len = 0;
INT8U   AT_IPR_info = 2; // 波特率数值



const INT8U MON[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
INT64U time_total_from_1970 = 0;



/**
* @brief  实现简单的10的次方函数
* @param[in] pown    10的pown次方
* @return 返回求取的次方结果
* @par 示例:
* @code
*        int num = pow_10_n(10);
* @endcode
* @deprecated 注意平方数目不要超过INT32U的范围
* @author       李晋南
* @date       2018/01/08
*/
INT32U pow_10_n( INT8U pown )
{
    INT32U num = 1;
    while( pown-- )
    {
        num *= 10;
    }
    return num;
}


/**
* @brief  ASCII数据转化为数字格式
* @param[in] pBuf       ASCII码指针的首地址
* @param[in] num        需要求得的数字的地址
* @param[in] len        需要读取判断的ASCII码的数据长度，从pBuf地址开始往后计算
* @return
* @par 示例:
* @code
*        ASCII_2_NUM_AT(pBuf, &num, 2);
* @endcode
* @deprecated 每一位ASCII码的范围都是'0' ~ ‘9’
* @author       李晋南
* @date       2018/01/08
*/

void ASCII_2_NUM_AT( INT8U* pBuf, INT32U* num, INT8U len ) // len 为ASCII长度
{
    INT8U i = 0;
    *num = 0;
    while( len )
    {
        *num += ( pBuf[i] - '0' ) * pow_10_n( len  - 1 );
        len --;
        i++;
    }
}

void ASCII_2_NUM_U8_AT( INT8U* pBuf, INT8U* num, INT8U len ) // len 为ASCII长度
{
    INT8U i = 0;
    *num = 0;
    while( len )
    {
        *num += ( pBuf[i] - '0' ) * pow_10_n( len  - 1 );
        len --;
        i++;
    }
}

/**
* @brief  ASCII数据转化为数字格式,INT16U 类型
* @param[in] pBuf       ASCII码指针的首地址
* @param[in] num        需要求得的数字的地址
* @param[in] len        需要读取判断的ASCII码的数据长度，从pBuf地址开始往后计算
* @return
* @par 示例:
* @code
*        ASCII_2_NUM_AT(pBuf, &num, 2);
* @endcode
* @deprecated 每一位ASCII码的范围都是'0' ~ ‘9’
* @author       李晋南
* @date       2018/01/08
*/

void ASCII_2_NUM_U16_AT( INT8U* pBuf, INT16U* num, INT8U len ) // len 为ASCII长度
{
    INT8U i = 0;
    *num = 0;
    while( len )
    {
        *num += ( pBuf[i] - '0' ) * pow_10_n( len  - 1 );
        len --;
        i++;
    }
}

/**
* @brief  数字格式转化为ASCII数据
* @param[in] num        转化的数字的值
* @param[in] pBuf       转化得到的ASCII码指针地址
* @param[in] len        转化后得到ASCII码的长度大小的指针
* @return
* @par 示例:
* @code
*        ASCII_2_NUM_AT(127, buf, &len);
* @endcode
* @deprecated 数据长度范围不要超过INT32U范围。指针长度防止出现异常段错误
* @author       李晋南
* @date       2018/01/08
*/
void NUM_2_ASCII_AT( INT32U num, INT8U* pBuf, INT8U* len )
{
    INT8U  i = 0, temp_len;
    INT32U temp_num = num;
    *len = 0;
    if( temp_num == 0 )
    {
        *len = 1;
    }
    while( temp_num )
    {
        *len += 1;
        temp_num /= 10;
    }
    temp_len = *len;
    while( temp_len )
    {
        pBuf[i] = num % pow_10_n( temp_len ) /  pow_10_n( temp_len - 1 ) +  '0';
        i++;
        temp_len--;
    }
    return;
}

/**
* @brief  HEX格式数据转化为ASCII码数据，例如 0x6800 转化为 “6800”
* @param[in] pBufhex        需要转化的hex数组首地址
* @param[in] pBuf           转化得到的ASCII码指针地址
* @param[in] hex_len        转化hex数组的长度
* @return
* @par 示例:
* @code
*        HEX_2_ASCII_AT(pBufhex, buf, len);
* @endcode
* @deprecated
* @author       李晋南
* @date       2018/01/08
*/
void HEX_2_ASCII_AT( INT8U* pBufhex, INT8U* pBuf,  INT16U hex_len )
{
    INT16U i;
    for( i = 0; i < hex_len; i++ )
    {
        if( ( ( pBufhex[i] & 0xf0 ) >> 4 ) > 9 )
        {
            pBuf[2 * i] = ( ( pBufhex[i] & 0xf0 ) >> 4 ) - 10 + 'A';
        }
        else
        {
            pBuf[2 * i] = ( ( pBufhex[i] & 0xf0 ) >> 4 ) + '0';
        }
        if( ( pBufhex[i] & 0x0f ) > 9 )
        {
            pBuf[2 * i +  1] = ( pBufhex[i] & 0x0f ) - 10 + 'A';
        }
        else
        {
            pBuf[2 * i + 1] = ( pBufhex[i] & 0x0f ) + '0';
        }
    }
    return;
}

/**
* @brief  ASCII码数据转化为HEX格式数据，例如 “6800” 转化为 0x6800
* @param[in] pBuf          需要转化的ASCII数组首地址
* @param[in] pBufhex       转化得到的HEX指针地址
* @param[in] hex_len       转化为hex数组的长度
* @return
* @par 示例:
* @code
*        ASCII_2_HEX_AT(pBufhex, buf, len);
* @endcode
* @deprecated
* @author       李晋南
* @date       2018/01/08
*/
void ASCII_2_HEX_AT( INT8U* pBuf, INT8U* pBufhex, INT16U hex_len )
{
    INT16U i, index;
    for( i = 0; i < hex_len; i++ )
    {
        index = 2 * i;
        if( ( pBuf[index] <= '9' ) && ( pBuf[index ] >= '0' ) )
        {
            pBufhex[i] = ( ( pBuf[index] - '0' ) << 4 );
        }
        else if( ( pBuf[index ] <= 'z' ) && ( pBuf[index ] >= 'a' ) )
        {
            pBufhex[i] = ( ( pBuf[index] - 'a' + 10 ) << 4 );
        }
        else if( ( pBuf[index ] <= 'Z' ) && ( pBuf[index] >= 'A' ) )
        {
            pBufhex[i] = ( ( pBuf[index] - 'A' + 10 ) << 4 );
        }
        index = 2 * i + 1;
        if( ( pBuf[index] <= '9' ) && ( pBuf[index ] >= '0' ) )
        {
            pBufhex[i] += pBuf[index ] - '0';
        }
        else if( ( pBuf[index ] <= 'z' ) && ( pBuf[index ] >= 'a' ) )
        {
            pBufhex[i] += pBuf[index] - 'a' + 10;
        }
        else if( ( pBuf[index ] <= 'Z' ) && ( pBuf[index] >= 'A' ) )
        {
            pBufhex[i] += pBuf[index] - 'A' + 10;
        }
    }
    return;
}

void mem_cpy( INT8U* dst, INT8U* src, INT16U len )
{
    while( len-- )
    {
        *( dst++ ) = *( src++ );
    }
}

INT64U mktime( unsigned int year, unsigned int mon,
               unsigned int day, unsigned int hour,
               unsigned int min, unsigned int sec )
{
    if( 0 >= ( int )( mon -= 2 ) )
    {
        /**//* 1..12 -> 11,12,1..10 */
        mon += 12;      /**//* Puts Feb last since it has leap day */
        year -= 1;
    }
    return ( ( (
                   ( INT64U )( year / 4 - year / 100 + year / 400 + 367 * mon / 12 + day ) +
                   year * 365 - 719499
               ) * 24 + hour /**/ /* now have hours */
             ) * 60 + min /**/ /* now have minutes */
           ) * 60 + sec; /**/ /* finally seconds */
}


void GetDateTimeFromSecond( unsigned long lSec, AT_time_t* tTime )
{
    INT16U i, j, iDay;
    unsigned long lDay;
    lDay = lSec / SECOND_OF_DAY;
    lSec = lSec % SECOND_OF_DAY;
    i = 1970;
    while( lDay > 365 )
    {
        if( ( ( i % 4 == 0 ) && ( i % 100 != 0 ) ) || ( i % 400 == 0 ) )
        {
            lDay -= 366;
        }
        else
        {
            lDay -= 365;
        }
        i++;
    }
    if( ( lDay == 365 ) && !( ( ( i % 4 == 0 ) && ( i % 100 != 0 ) ) || ( i % 400 == 0 ) ) )
    {
        lDay -= 365;
        i++;
    }
    tTime->year = i;
    for( j = 0; j < 12; j++ )
    {
        if( ( j == 1 ) && ( ( ( i % 4 == 0 ) && ( i % 100 != 0 ) ) || ( i % 400 == 0 ) ) )
        {
            iDay = 29;
        }
        else
        {
            iDay = MON[j];
        }
        if( lDay >= iDay )
        {
            lDay -= iDay;
        }
        else
        {
            break;
        }
    }
    tTime->month = j + 1;
    tTime->day = lDay + 1;
    tTime->hour = ( lSec / 3600 ) % 24; //这里注意，世界时间已经加上北京时间差8，
    tTime->min = ( lSec % 3600 ) / 60;
    tTime->sec = ( lSec % 3600 ) % 60;
}


void uart_Answer_OK( void )
{
    drv_UartSend( "\rOK\r", 4 );
}



void uart_Answer_ERROR( void )
{
    drv_UartSend( "\rERROR\r", 7 );
}

void uart_Answer_Invalid_Command( void )
{
    drv_UartSend( "\rInvalid command\r", 17 );
}

void uart_Answer_ERRORn( INT8U num )
{
    INT8U ERROR_buf[11] = {'\r', 'E', 'R', 'R', 'O', 'R', ':'}, ERROR_len = 8;
    NUM_2_ASCII_AT( num, ERROR_buf + 7, &ERROR_len );
    ERROR_buf[ERROR_len + 7] = '\r';
    drv_UartSend( ERROR_buf, ERROR_len + 8 );
}


void uart_Answer_AT( void )
{
    drv_UartSend( "\rAT:1\r", 5 ); // AT: 1  1 AT指令版本号
}
/**
* @brief   硬件初始化，CAC复位
* @param[in]  无
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN01_Fn1_AT_HINT( void )
{
    uart_Answer_OK();
    drv_Delay10ms( 20 );                                                                       //  等待发送完毕
    cac_SaveAll();
    drv_Resetcac();                                                                                  //  CAC复位
}


/**
* @brief   参数初始化，清除所有DAU档案
* @param[in]  无
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN01_Fn2_F3_AT_PINT( void )
{
    cac_ClearAll();
    uart_Answer_OK();
    cac_SaveAll();
    drv_Delay10ms( 20 );
}


/**
* @brief   抄读表信息
* @param[in]  无
* @author       李晋南
* @date       2018/01/15
*/
void DL2013_AFN03_Fn1_F101_AT_RTMN( INT8U* pBuf ) 
{
    INT16U len;
    INT8U Dau_Addr[6], protocol_type, cmd_type, *Get645_data;
    INT8U  i, flag_get_all_num;
    INT16U tmpDAUNum = 0;
    INT8U tmpFrameType = 0;
    if( ( pBuf[20] != ',' ) || ( pBuf[22] != ',' ) || ( pBuf[24] != ',' ) )
    {
        uart_Answer_ERROR();
        return;
    }
    for( i = 0 ; i < 4; i++ )
    {
        if( pBuf[26 + i] == ',' )
        {
            ASCII_2_NUM_U16_AT( pBuf + 25, &len, i + 1 );
            if( len > 200 )
            {
                uart_Answer_ERROR();
                return;
            }
            flag_get_all_num   = 1;
            if( pBuf[27 + i + len] == '\r' )
            {
                if( flag_get_all_num == 1 )
                {
                    flag_get_all_num = 2;
                }
                Get645_data = malloc( len );
                mem_cpy( Get645_data, pBuf + 27 + i, len );
                break;
            }
        }
    }
    if( flag_get_all_num != 2 )
    {
        uart_Answer_ERROR();
        return;
    }
    ASCII_2_HEX_AT( pBuf + 8,      Dau_Addr,       6 );
    ASCII_2_NUM_U8_AT( pBuf + 21,  &protocol_type,   1 );
    ASCII_2_NUM_U8_AT( pBuf + 23,  &cmd_type,        1 );
    if( protocol_type == 0 )
    {
        backMeter = 1;
    }
    else
    {
        backMeter = 3;
    }
    //  已有抄表任务
    if( mCAC.bMeter == TRUE1 )
    {
        //drv_Printf("\n已有抄表任务");
        free( Get645_data );
        uart_Answer_ERROR(); // 需改动为其他错误码
        return ;                                                                //  返回主节点忙
    }
    //  查找DAU序号
    tmpDAUNum = cac_CheckDAUNum( Dau_Addr );
    //  drv_Printf( "\n目标DAU[%3d] =  ", tmpDAUNum );
    drv_PrintfDAU( pDAU );
    //  节点不在档案
    if( tmpDAUNum >= MAX_DAU )
    {
        // drv_Printf("\n节点不在档案");
        free( Get645_data );
        uart_Answer_ERROR(); // 需改动为其他错误码  //  返回不在网
        return ;
    }
    //  若为透传命令
    if( cmd_type == 0x00 )
    {
        //  更新DAU协议类型
        mDAU[tmpDAUNum].aDProType[0] = Get645FrameType( Get645_data );
    }
    //  若有协议字段
    else
    {
        //  解析报文帧类型
        tmpFrameType = Get645FrameType( Get645_data );
        mDAU[tmpDAUNum].aDProType[0] = tmpFrameType;
    }
    // drv_Printf("\n==============================启动抄表=============================");
    if( protocol_type == 0 )
    {
        set_Meter( tmpDAUNum, Get645_data, len, mDAU[tmpDAUNum].aDProType[0], 0, RF_ASK_DLMS_DATA );
    }
    else
    {
        set_Meter( tmpDAUNum, Get645_data, len, mDAU[tmpDAUNum].aDProType[0], 0, RF_ASK_METER_DATA );
    }
    free( Get645_data );
}

/**
* @brief    查询厂商代码和版本信息
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN03_Fn1_AT_MVINFO( INT8U* pBuf )
{
    INT8U i = 0, temp_buf[3];
    pBuf[i++] = '\r';
    mem_cpy( pBuf + i, "friendcom", 9 );
    i += 9;
    pBuf[i++] = ',';
    temp_buf[0] = DT_YEAR;
    temp_buf[1] = DT_MOTH;
    temp_buf[2] = DT_DATA;
    HEX_2_ASCII_AT( temp_buf, pBuf + i, 3 );
    i += 6;
    pBuf[i++] = ',';
    temp_buf[0] = DT_EDITIOM_H;
    HEX_2_ASCII_AT( temp_buf, pBuf + i, 1 );
    i += 2;
    pBuf[i++] = '.';
    temp_buf[0] = DT_EDITIOM_L;
    HEX_2_ASCII_AT( temp_buf, pBuf + i, 1 );
    i += 2;
    pBuf[i++] = ',';
    temp_buf[0] = HARD_VERSION_AT;
    HEX_2_ASCII_AT( temp_buf, pBuf + i, 1 );
    i += 2;
    pBuf[i++] = '\r';
    drv_UartSend( pBuf, i );
}

/**
* @brief    查询主节点地址
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN03_Fn4_AT_HNA( INT8U* pBuf )
{
    pBuf[0] = '\r';
    //  写CAC地址到数据单元
    HEX_2_ASCII_AT( mCAC.aCAddr, pBuf + 1, 6 );
    pBuf[13] = '\r';
    drv_UartSend( pBuf, 14 );
}

/**
* @brief    查询主节点状态字和通信速率
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN03_Fn5_AT_SCCR( INT8U* pBuf )
{
    INT8U  status_world[2], temp_len1;
    //  主节点状态字
    // 周期抄表模式=01b                          01   仅集中器主导(无线); 10   仅路由主导(载波) ; 11 两种都支持(双模异构)
    //  主节点信道特征 =00b                 00 微功率无线；01 单相供电单相传输；10 单供三传；11三供三传
    //  通信速率数量 =  1（0001b）
    //  pBuf[1] 高四位备用，低四位信道数量
    status_world[0] = 0x41; //
    status_world[1] = 0x1f; //  信道实际数量32
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( status_world[1], pBuf + 1, &temp_len1 ); //信道实际数量
    pBuf[temp_len1 + 1] = '\r';
    // 返回数据
    drv_UartSend( pBuf, temp_len1 + 2 );
}

/**
* @brief      读取从节点监控最大超时时间
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN03_Fn7_AT_NMMT( INT8U* pBuf )
{
    INT8U len = 0;
    pBuf[0] = '\r';
    //  写CAC地址到数据单元
    NUM_2_ASCII_AT( mCAC.i1ReadMeterMaxTime, pBuf + 1, &len );
    pBuf[len + 1] = '\r';
    drv_UartSend( pBuf, len + 2 );
}


/**
* @brief      查询无线通信参数
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN03_Fn8_AT_HNCP( INT8U* pBuf )
{
    INT8U temp_len1 = 0;
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( mCAC.i1BigCHXX, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    pBuf[temp_len1 + 2] = mCAC.i1RFPower + '0';
    pBuf[temp_len1 + 3] = '\r';
    drv_UartSend( pBuf, temp_len1 + 4 );
}

/**
* @brief       查询通信延时相关的广播时长
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN03_Fn9_AT_BRDDT( INT8U* pBuf )
{
    INT8U temp_len1 = 0;
    INT16U DelayTime = cac_GetBroadDelayTime( mCAC.i2AllNum + 5 ); //cac_GetBroadDelayTime(mCAC.i2ShiXi);//
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( DelayTime, pBuf + 1,  &temp_len1 ); //  广播延时根据时隙计算
    pBuf[temp_len1 + 1] = '\r';
    drv_UartSend( pBuf, temp_len1 + 2);
}


/*
INT8U Data03HF10_(INT8U *xBuf)
{
    INT8U i;

    // 周期抄表模式=01 b                          01   仅集中器主导(无线); 10   仅路由主导(载波) ; 11 两种都支持(双模异构)
    // 从节点信息模式=1 b                       需要下发从节点信息
    // 路由管理方式=1 b                            本地模块有路由功能
    // 通信方式=0011 b                                 1  窄带载波; 2  宽带载波; 3  无线微功率; 其余  保留
    xBuf[0] = 0x73;                                                                                          //  01 1 1 0011

    // 广播命令信道执行方式=00 b     广播命令不需信道标识
    // 广播命令确认方式=1 b                  广播完成前返回确认报文（确认报文中''等待执行时间”有效）
    // 失败节点切换发起方式=10 b     10 集中器控制; 01  本地模块自主切换
    // 传输延时参数支持=110 b             依次表示广播、从节点监控、路由主动抄表向集中器提供传输
    //                                                                      时延参数的情况，1表支持
    xBuf[1]=0x36;

    // 低压电网掉电信息= 000b            依次代表A, B, C三相掉电信息，0表未掉电
    // 信道数量= 31(最大值)
    xBuf[2]=0x1f;

    //  高4位保留
    //  低4位 速率数量=1
    xBuf[3] = 0x01;

    // 保留字段，写0
    xBuf[4]=0;
    xBuf[5]=0;

    // 从节点监控最大超时时间
    xBuf[6]     =   mCAC.i1ReadMeterMaxTime;

    // 广播命令最大超时时间
    xBuf[7]     =   (INT8U)TIME_MAX_BROADCAST;
    xBuf[8]     =   (INT8U)(TIME_MAX_BROADCAST >> 8);

    // 最大支持报文长度
    xBuf[9]     =   255;
    xBuf[10]    =   0;

    // 文件传输支持的最大单个数据包长度
    xBuf[11]    =   128;
    xBuf[12]    =   0;

    // 升级操作等待时间
    xBuf[13]    =   TIME_MAX_UPDATE;

    // 主节点地址
    xBuf[14]=mCAC.aCAddr[0];
    xBuf[15]=mCAC.aCAddr[1];
    xBuf[16]=mCAC.aCAddr[2];
    xBuf[17]=mCAC.aCAddr[3];
    xBuf[18]=mCAC.aCAddr[4];
    xBuf[19]=mCAC.aCAddr[5];

    // 支持最大从节点数量
    xBuf[20]=(INT8U)MAX_DAU;
    xBuf[21]=(INT8U)(MAX_DAU>>8);

    // 当前从节点数量
    xBuf[22]=(INT8U)mCAC.i2GoodDAU;
    xBuf[23]=(INT8U)(mCAC.i2GoodDAU>>8);

    // 通信模块使用的协议发布日期
    xBuf[24]=0x21;
    xBuf[25]=0x03;
    xBuf[26]=0x13;

    // 通信模块使用的协议备份发布日期
    xBuf[27]=0x21;
    xBuf[28]=0x03;
    xBuf[29]=0x13;

    // 厂家信息标示
    xBuf[30]='C';
    xBuf[31]='F';
    xBuf[32]='M';
    xBuf[33]='J';
    xBuf[34]=DT_DATA;
    xBuf[35]=DT_MOTH;
    xBuf[36]=DT_YEAR;
    xBuf[37]=DT_EDITIOM_L;
    xBuf[38]=DT_EDITIOM_H;

    //  通信速率组信息
    //  D15: 速率单位标示; 1表示Kbps，0表示bps
    //  D14~D0 通信速率=0默认，
    for(i=0;i<xBuf[3];i++)
    {
        xBuf[39+i*2]=0;
        xBuf[40+i*2]=0x80;
    }

    return (xBuf[3] *2 + 39);
}
*/

/**
* @brief           查询本地通信模块运行模式
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN03_Fn10_AT_HMOD( INT8U* pBuf )  //待续
{
    INT8U length = 0;
    //length = Data03HF10(pBuf);                                                                  //  写用户数据单元
    drv_UartSend( pBuf, length );
}



/**
* @brief         查询场强门限
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN03_Fn100_AT_RSSIT( INT8U* pBuf )
{
    //  场强门限，取值50~120
    INT8U len  = 0;
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( mCAC.i1Valve, pBuf + 1, &len );
    pBuf[len + 1] = '\r';
    drv_UartSend( pBuf, len + 2 );
}


/**
* @brief         查询中心节点时间
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN03_Fn101_AT_RTC( INT8U* pBuf )
{
    INT8U i = 0;
    AT_time_t time_now;
    GetDateTimeFromSecond( time_total_from_1970, &time_now );
    time_now.year -= 2000;
    if( ( time_now.year <= 0 ) || ( time_now.year > 2038 ) )
    {
        time_now.year = 0;
    }
    pBuf[i++] = '\r';
    pBuf[i++] = time_now.year / 10 + '0';
    pBuf[i++] = time_now.year % 10 + '0';
    pBuf[i++] = time_now.month / 10 + '0';
    pBuf[i++] = time_now.month % 10 + '0';
    pBuf[i++] = time_now.day / 10 + '0';
    pBuf[i++] = time_now.day % 10 + '0';
    pBuf[i++] = time_now.hour / 10 + '0';
    pBuf[i++] = time_now.hour % 10 + '0';
    pBuf[i++] = time_now.min / 10 + '0';
    pBuf[i++] = time_now.min % 10 + '0';
    pBuf[i++] = time_now.sec / 10 + '0';
    pBuf[i++] = time_now.sec % 10 + '0';
    pBuf[i++] = '\r';
    drv_UartSend( pBuf, i );
}



/**
* @brief         链路接口发送测试
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN04_Fn1_AT_TSST( INT8U* pBuf )
{
    INT16U timeout = 0;
    INT8U  i;
    
    
    INT8U Buf[35] = {0x22,
                     0x41, 0xcd,
                     0x0f,
                     0xff, 0xff, // panid
                     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // buf[6]
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, // cac addr  12
                     0x3c,
                     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //  buf[19]
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, // car addr 25
                     1, 1, 0, 0x31
                     // should add other , such as: datalen + data
                    };
    INT8U  temp_buf[13];
    
    for( i = 0 ; i < 5; i++ )
    {
        if( pBuf[9 + i] == '\r' )
        {
            if( i > 2 )
            {
                uart_Answer_ERRORn( ERROR_04H_F1_AT_TIMEOUT_OVER_FLOW );
                return;
            }
            ASCII_2_NUM_AT( &( pBuf[8] ), ( INT32U* )&timeout, i + 1 );
            if( timeout > 255 )
            {
                uart_Answer_ERRORn( ERROR_04H_F1_AT_TIMEOUT_OVER_FLOW );
                return;
            }
            break;
        }
    }
    
    mem_cpy( Buf + 12, mCAC.aCAddr, 6 );
    mem_cpy( Buf + 25, mCAC.aCAddr, 6 );
    temp_buf[0] = 12;
    HEX_2_ASCII_AT(mCAC.aCAddr,temp_buf + 1,6);
    mem_cpy( pBuf, Buf, 0x23 );
    mem_cpy( pBuf + 0x23, temp_buf, 13);
    pBuf[0] += 13;
    
    
    for(i=0;i<timeout;i++)
    {
        drv_stm32WDT();
        drv_RfSend( pBuf, CH_TYPE_TEST_0 );
        drv_Delay10ms(100);
    }
    
    uart_Answer_OK();
}




/**
* @brief        链路接口从节点点名
* @param[in]
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN04_Fn2_AT_NRC( void )
{
    set_call_all();
    if( tsk_call_all() )
    {
        uart_Answer_OK();
    }
    else
    {
        uart_Answer_ERROR();
    }
}


/**
* @brief         设置主节点地址
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN05_Fn1_AT_HNA( INT8U* pBuf )
{
    INT8U i = 0, addr_buf[6];
    ASCII_2_HEX_AT( pBuf + 7, addr_buf, 6 );
    // 若主节点地址变动
    if( memcmp( mCAC.aCAddr, addr_buf, 6 ) != 0 )
    {
        // 提取CAC地址
        for( i = 0; i < 6; i++ )
        {
            mCAC.aCAddr[i] = addr_buf[i] ;
        }
        // 计算panID
        LongToPanID( mCAC.aCAddr, mCAC.panID );
        // 计算实际工作频道
        mCAC.i1BigCH = ( INT8U )( ( mCAC.panID[0] % ( SUM_CH_NUM - 1 ) ) + 1 );
        drv_setCH( mCAC.i1BigCH );
        // 需要根据底噪自动工作频道
        mCAC.bNewWrkCH  =   TRUE1;
        // 保存CAC变更信息
        cac_SaveCAC();
    }
    uart_Answer_OK();
}

/**
* @brief         允许/禁止从节点上报
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN05_Fn2_AT_NEREP( INT8U* pBuf )
{
    if( pBuf[9] > '1' )
    {
        uart_Answer_ERROR();
        return;
    }
    else if( pBuf[9] == '1' )
    {
        mCAC.bReport = TRUE1;    //  允许上报
    }
    else if( pBuf[9] == '0' )
    {
        mCAC.bReport = FALSE0;    //  禁止上报
    }
    uart_Answer_OK();                                                                                       // 返回确认
}

/**
* @brief         启动广播
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08 初稿 & 2018/01/15 完善
*/
void DL2013_AFN05_Fn3_AT_SBRD( INT8U* pBuf )
{
    gDtMeter* tM = ( gDtMeter* )( mRf.aBuf );
    INT8U year, month, day, hour, minute, sec, i;
    INT8U conformation_buf[20] = {0x01, 0x12,
                                  0x68, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x68, 0x08,
                                  0x06, //    buf[11] 长度
                                  0x37, 0x53, 0x44, 0x48, 0x34, 0x4B, //   buf[12] 秒 分 时， 日 月 年 +33
                                  0x09, //    buf[18]
                                  0x16
                                 };
    ASCII_2_NUM_U8_AT( pBuf + 11, &year, 2 );
    ASCII_2_NUM_U8_AT( pBuf + 13, &month, 2 );
    ASCII_2_NUM_U8_AT( pBuf + 15, &day, 2 );
    if( IsLegal( year, month, day ) == 0 )
    {
        uart_Answer_ERRORn( ERROR_05H_F3_AT_TIME_OVER_FLOW );
        return;
    }
    ASCII_2_NUM_U8_AT( pBuf + 17, &hour, 2 );
    if( hour > 24 )
    {
        uart_Answer_ERRORn( ERROR_05H_F3_AT_TIME_OVER_FLOW );
        return;
    }
    ASCII_2_NUM_U8_AT( pBuf + 19, &minute, 2 );
    if( minute > 60 )
    {
        uart_Answer_ERRORn( ERROR_05H_F3_AT_TIME_OVER_FLOW );
        return;
    }
    ASCII_2_NUM_U8_AT( pBuf + 21, &sec, 2 );
    if( sec > 60 )
    {
        uart_Answer_ERRORn( ERROR_05H_F3_AT_TIME_OVER_FLOW );
        return;
    }
    conformation_buf[18] = 0;
    for( i = 2; i < 18; i++ )
    {
        conformation_buf[18] += conformation_buf[i];
    }
    conformation_buf[12] = sec + 0x30;
    conformation_buf[13] = minute + 0x30;
    conformation_buf[14] = hour + 0x30;
    conformation_buf[15] = day + 0x30;
    conformation_buf[16] = month + 0x30;
    conformation_buf[17] = year + 0x30;
    conformation_buf[18] = 0;
    for( i = 2; i < 18; i++ )
    {
        conformation_buf[18] += conformation_buf[i ];
    }
    tM->i1Baud = ( INT8U )( mCAC.i2BrdNum++ );
    rf_BroadcastTime( &( conformation_buf[2] ), conformation_buf[1] );                                 //  广播校时
    drv_SetTimeMeter( ( INT16U )( xb_Capacity( mCAC.i2DownDAU ) ) );             //  动态规模
    mCAC.bMeter = TRUE1;
    mCAC.bBroad = TRUE1;
    uart_Answer_OK();// 返回确认
}


/**
* @brief          设置从节点监控最大超时时间
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN05_Fn4_AT_NMMT( INT8U* pBuf )
{
    INT8U i = 0 ;
    INT16U timeout;
    for( i = 0 ; i < 5; i++ )
    {
        if( pBuf[9 + i] == '\r' )
        {
            if( i > 2 )
            {
                uart_Answer_ERRORn( ERROR_05H_F4_AT_TIMEOUT_OVER_FLOW );
                return;
            }
            ASCII_2_NUM_AT( &( pBuf[8] ), ( INT32U* )&timeout, i + 1 );
            if( timeout > 255 )
            {
                uart_Answer_ERRORn( ERROR_05H_F4_AT_TIMEOUT_OVER_FLOW );
                return;
            }
            mCAC.i1ReadMeterMaxTime =  timeout;
            uart_Answer_OK();                                                                                            // 返回确认
            return;
        }
    }
    uart_Answer_Invalid_Command();
}

/**
* @brief          设置无线通信参数
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN05_Fn5_AT_HNCP( INT8U* pBuf )
{
    INT8U  speed, Power;
    INT8U i;
    for( i = 0 ; i < 3; i++ )
    {
        if( ( pBuf[11 + i] == '\r' ) && ( pBuf[9 + i] == ',' ) )
        {
            ASCII_2_NUM_AT( &( pBuf[8] ), ( INT32U* )&speed, i + 1 );
            ASCII_2_NUM_AT( &( pBuf[10 + i] ), ( INT32U* )&Power, 1 );
            // 功率只有四档
            if( Power <= 4 )
            {
                mCAC.i1RFPower = Power;
            }
            else
            {
                uart_Answer_ERRORn( ERROR_05H_F5_AT_POWER_OVER_FLOW );
            }
            // 自动选择或默认：不改变
            if( speed >= 254 )                                                                              //  254自动选择
            {
                uart_Answer_OK();
            }
            else if( ( speed < SUM_CH_NUM ) && ( speed > 0 ) )                                            //  1~32     // 1 ~ 32，更新保存频道
            {
                mCAC.i1BigCHXX = speed;
                cac_SaveCAC();
                uart_Answer_OK();
            }
            else                                                                                                 //  其余
            {
                uart_Answer_ERRORn( ERROR_05H_F5_AT_SPEED_OVER_FLOW );
            }
            return;
        }
    }
    uart_Answer_Invalid_Command();
}


/**
* @brief           设置场强门限
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN05_Fn100_AT_RSSIT( INT8U* pBuf )
{
    INT8U rssi_num = 0;
    if( pBuf[11] == '\r' )
    {
        ASCII_2_NUM_AT( pBuf + 9, ( INT32U* )&rssi_num, 2 );
    }
    else if( pBuf[12] == '\r' )
    {
        ASCII_2_NUM_AT( pBuf + 9, ( INT32U* )&rssi_num, 3 );
    }
    if( ( rssi_num < MIN_RISS ) || ( rssi_num > MAX_RISS ) )
    {
        uart_Answer_ERRORn( ERROR_05H_F100_AT_RSSIT_OVER_FLOW );
    }
    else
    {
        mCAC.i1Valve = rssi_num;
        cac_SaveCAC();
        uart_Answer_OK();                                                                                       // 返回确认
    }
}


static INT8U IsLeapYear( int year )
{
    if( ( ( year % 4 == 0 ) && ( year % 100 != 0 ) ) || ( year % 400 == 0 ) )
    {
        return 1;
    }
    return 0;
}
INT8U IsLegal( int year, int mon, int day )
{
    //大：1 3 5 7 8 10 12
    //小：4 6 9 11
    //平：2
    if( year < 0 || mon <= 0 || mon > 12 || day <= 0 || day > 31 )
    {
        return 0;
    }
    if( 1 == mon || 3 == mon || 5 == mon || 7 == mon || 8 == mon || 10 == mon || 12 == mon )
    {
        return 1;
    }
    if( IsLeapYear( year ) )
    {
        if( 2 == mon && ( 28 == day || 30 == day || 31 == day ) )
        {
            return 0;
        }
        return 1;
    }
    else
    {
        if( 2 == mon && ( 29 == day || 30 == day || 31 == day ) )
        {
            return 0;
        }
        return 1;
    }
}
/**
* @brief             设置中心节点时间
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void AT_RTC_REQ( void )
{
    INT8U buf[10] = "\rAT+RTC?\r";
    drv_UartSend( buf, 9 );
}

void DL2013_AFN05_Fn101_AT_RTC( INT8U* pBuf )
{
    INT16U year = 0;
    INT8U month, day, hour, minute, sec;
    ASCII_2_NUM_U16_AT( pBuf + 7, &year, 2 );
    year += 2000;
    ASCII_2_NUM_U8_AT( pBuf + 9, &month, 2 );
    ASCII_2_NUM_U8_AT( pBuf + 11, &day, 2 );
    if( IsLegal( year, month, day ) == 0 )
    {
        uart_Answer_ERRORn( ERROR_05H_F101_AT_TIME_OVER_FLOW );
        return;
    }
    ASCII_2_NUM_U8_AT( pBuf + 13, &hour, 2 );
    if( hour > 24 )
    {
        uart_Answer_ERRORn( ERROR_05H_F101_AT_TIME_OVER_FLOW );
        return;
    }
    gSysTime.hour = hour;    // 设置小时
    ASCII_2_NUM_U8_AT( pBuf + 15, &minute, 2 );
    if( minute > 60 )
    {
        uart_Answer_ERRORn( ERROR_05H_F101_AT_TIME_OVER_FLOW );
        return;
    }
    gSysTime.count = 60 * minute;
    ASCII_2_NUM_U8_AT( pBuf + 17, &sec, 2 );
    if( sec > 60 )
    {
        uart_Answer_ERRORn( ERROR_05H_F101_AT_TIME_OVER_FLOW );
        return;
    }
    gSysTime.count += sec;                                             // 1h = 60min = 3600s
    if( ( gSysTime.hour == 0x21 ) && ( TskOptimizeLimitTime == 0 ) )
    {
        if( mCAC.bSetup != TRUE1 && nPrioIIFlag.fHourSate != 2 )                                                             // 有新增节点
        {
            nPrioIIFlag.fHourSate = 2;
            drv_SetTimeDown( 10 );
        }
    }
    time_total_from_1970 = mktime( year, month, day, hour, minute, sec );
    uart_Answer_OK();
}



/**
* @brief              查询从节点数量
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/

void DL2013_AFN10_Fn1_AT_NNUM( INT8U* pBuf )
{
    INT8U temp_len1 = 0, temp_len2 = 0;
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( mCAC.i2GoodDAU, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    NUM_2_ASCII_AT( MAX_DAU, pBuf + temp_len1 + 2,  &temp_len2 );
    pBuf[temp_len1 + temp_len2 + 2] = '\r';
    drv_UartSend( pBuf, temp_len1 + temp_len2  + 3 );
    gFlgUrtCmpFile  =   TRUE1;
}

/**
* @brief      查询从节点信息
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/09
*/
void DL2013_AFN10_Fn2_AT_NINFO( INT8U* pBuf )
{
    INT16U tN1;
    INT16U tN2;
    INT16U i, pBuf_index;
    INT16U start_serial_number = 0, serial_number_num = 0;
    INT8U flag_get_all_num = 0, serial_number_len;
    INT8U temp_len1, relay_level = 0;
    for( i = 0 ; i < 10; i++ )
    {
        if( pBuf[9 + i] == ',' )
        {
            ASCII_2_NUM_U16_AT( pBuf + 8, &start_serial_number, i + 1 );
            serial_number_len = i + 1;
            flag_get_all_num   = 1;
        }
        else if( pBuf[9 + i] == '\r' )
        {
            ASCII_2_NUM_U16_AT( pBuf + 9 + serial_number_len, &serial_number_num, i  - serial_number_len );
            if( flag_get_all_num == 1 )
            {
                flag_get_all_num = 2;
            }
            break;
        }
    }
    if( flag_get_all_num != 2 )
    {
        uart_Answer_ERROR();
        return;
    }
    if( serial_number_num > 25 )
    {
        uart_Answer_ERRORn( ERROR_10H_F2_AT_NODE_OVER_FLOW );
        return;
    }
    pBuf[0] = '\r';
    // 组网节点个数
    NUM_2_ASCII_AT( serial_number_num, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    pBuf_index = temp_len1 + 2;
    tN1  = 1;
    tN2  = start_serial_number;                                                                                //  tN2保存从节点起始序号
    for( i = 0; i < mCAC.i2AllNum; i++ )                                                         // 遍历从节点队列
    {
        if( uart_DAUType( i, 3 ) == TRUE1 )                                                     //  若是下载节点，tN1加1
        {
            if( tN1 >= tN2 )                                                                                  //  找到起始序号，跳出循环
            {
                break;
            }
            tN1++;
        }
    }
    tN2 = 0;
    for( ; i < mCAC.i2AllNum; i++ )                                                                //  继续遍历从节点序列
    {
        if( uart_DAUType( i, 3 ) == TRUE1 )                                                     //  若是下载节点，tN2加1
        {
            HEX_2_ASCII_AT( mDAU[i].aDAddr, pBuf + pBuf_index,  6 );
            pBuf_index += 12;
            pBuf[pBuf_index ++] =  ',';
            if( dauF_Good( i ) == TRUE1 )                                                        //  830台体接口:  0~6表中继层次，0x0f表不在网
            {
                relay_level = mDAU[i].b4DLayerSon - 1;
            }
            else
            {
                relay_level = 9;
            }
            pBuf[pBuf_index ++] = '0' + relay_level % 10 ;
            pBuf[pBuf_index ++] = ',';
            pBuf[pBuf_index ++] = '0' + ( INT8U )( mDAU[i].aDProType[1] );           // 通信协议类型 comm_type
            pBuf[pBuf_index ++] = ',';                                                    // 协议类型:  档案类型
            tN2++;
            if( tN2 >= serial_number_num || tN2 >= 25 )                                                   // 若tN2 达到25或要求查询数量，退出循环
            {
                break;
            }
        }
    }
    if( temp_len1 == 2 )
    {
        if( tN2 / 10 == 0 )
        {
            pBuf[1] = '0';
            pBuf[2] = '0' + tN2;
        }
        else
        {
            NUM_2_ASCII_AT( tN2, pBuf + 1,  &temp_len1 );
        }
    }
    else  if( temp_len1 == 1 )
    {
        pBuf[1] = '0' + tN2;
    }
    pBuf[--pBuf_index] = '\r';
    drv_UartSend( pBuf, pBuf_index + 1 );
    if( FALSE0 == mCAC.bSetup && FALSE0 == mCAC.bOptimize && TRUE1  == gFlgUrtCmpFile )         // CAC未组网且未维护且开始档案同步
    {
        //  drv_Printf("\n本次下载%d个节点，其中有新增节点", pBuf[2]);
        drv_SetTimeDown( 40 );
    }
}

/**
* @brief               查询路由运行状态
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN10_Fn4_AT_RTST( INT8U* pBuf )
{
    INT8U temp_len1, temp_len2, work_switch = 0;
    pBuf[0] = '\r';
    // 组网节点个数
    NUM_2_ASCII_AT( mCAC.i2DownDAU, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    // 在网节点个数
    NUM_2_ASCII_AT( mCAC.i2GoodDAU, pBuf + temp_len1 + 2,  &temp_len2 );
    pBuf[temp_len1 + temp_len2 + 2] = ',';
    //  写工作开关格式
    if( mCAC.bYuLi == TRUE1 )                                                                               //  搜表
    {
        work_switch += 2;
    }
    if( mCAC.bSetup == TRUE1 )                                                                            //  组网
    {
        work_switch += 1;
    }
    pBuf[temp_len1 + temp_len2  + 3] = work_switch +  '0';
    pBuf[temp_len1 + temp_len2 + 4] = '\r';
    // 返回数据
    drv_UartSend( pBuf, temp_len1 + temp_len2 + 5 );
}



/**
* @brief         查询未抄读成功从节点信息
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/09
*/

void DL2013_AFN10_Fn5_AT_NLST( INT8U* pBuf )
{
    INT16U tN1;
    INT16U tN2;
    INT16U i, pBuf_index;
    INT16U start_serial_number = 0, serial_number_num = 0;
    INT8U flag_get_all_num = 0, serial_number_len;
    INT8U temp_len1;
    for( i = 0 ; i < 10; i++ )
    {
        if( pBuf[9 + i] == ',' )
        {
            ASCII_2_NUM_U16_AT( pBuf + 8, &start_serial_number, i + 1 );
            serial_number_len = i + 1;
            flag_get_all_num   = 1;
        }
        else if( pBuf[9 + i] == '\r' )
        {
            ASCII_2_NUM_U16_AT( pBuf + 9 + serial_number_len, &serial_number_num, i  - serial_number_len );
            if( flag_get_all_num == 1 )
            {
                flag_get_all_num = 2;
            }
            break;
        }
    }
    if( flag_get_all_num != 2 )
    {
        uart_Answer_ERROR();
        return;
    }
    if( serial_number_num > 25 )
    {
        uart_Answer_ERRORn( ERROR_10H_F5_AT_NODE_OVER_FLOW );
        return;
    }
    tN1  = 1;
    tN2  = start_serial_number;                                                                                  //  tN2保存从节点起始序号
    for( i = 0; i < mCAC.i2AllNum; i++ )                                                         // 遍历从节点队列
    {
        if( uart_DAUType( i, 2 ) == TRUE1 )                                                     //  若是不在网节点，tN1加1
        {
            if( tN1 >= tN2 )                                                                                  //  找到起始序号，跳出循环
            {
                break;
            }
            tN1++;
        }
    }
    tN2 = 0;
    pBuf[0] = '\r';
    // 组网节点个数
    NUM_2_ASCII_AT( serial_number_num, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    pBuf_index = temp_len1 + 2;
    for( ; i < mCAC.i2AllNum; i++ )                                                                //  继续遍历从节点序列
    {
        if( uart_DAUType( i, 2 ) == TRUE1 )                                                     //  若是不在网、故障节点，tN2加1
        {
            //   mDAU[i].aDAddr[0] =   mDAU[i].aDAddr[1] =   mDAU[i].aDAddr[2] =   mDAU[i].aDAddr[3] =   mDAU[i].aDAddr[4] =   mDAU[i].aDAddr[5] = 0x66;
            HEX_2_ASCII_AT( mDAU[i].aDAddr, pBuf + pBuf_index,  6 );
            pBuf_index += 12;
            pBuf[pBuf_index ++] =  ',';
            tN2++;
            if( tN2 >= serial_number_num || tN2 >= 25 )                                                       //  若tN2 达到25或要求查询数量，退出循环
            {
                break;
            }
        }
    }
    if( temp_len1 == 2 )
    {
        if( tN2 / 10 == 0 )
        {
            pBuf[1] = '0';
            pBuf[2] = '0' + tN2;
        }
        else
        {
            NUM_2_ASCII_AT( tN2, pBuf + 1,  &temp_len1 );
        }
    }
    else  if( temp_len1 == 1 )
    {
        pBuf[1] = '0' + tN2;
    }
    pBuf[--pBuf_index] = '\r';
    drv_UartSend( pBuf, pBuf_index + 1 );
}
/**
* @brief       查询主动注册从节点信息
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN10_Fn6_AT_AREG( INT8U* pBuf ) // 0108已验证，待实际测试方法测试
{
    INT16U tN1;
    INT16U tN2;
    INT16U i, pBuf_index;
    INT16U start_serial_number = 0, serial_number_num = 0;
    INT8U flag_get_all_num = 0, serial_number_len;
    INT8U temp_len1, relay_level = 0;
    for( i = 0 ; i < 10; i++ )
    {
        if( pBuf[9 + i] == ',' )
        {
            ASCII_2_NUM_U16_AT( pBuf + 8, &start_serial_number, i + 1 );
            serial_number_len = i + 1;
            flag_get_all_num   = 1;
        }
        else if( pBuf[9 + i] == '\r' )
        {
            ASCII_2_NUM_U16_AT( pBuf + 9 + serial_number_len, &serial_number_num, i  - serial_number_len );
            if( flag_get_all_num == 1 )
            {
                flag_get_all_num = 2;
            }
            break;
        }
    }
    if( flag_get_all_num != 2 )
    {
        uart_Answer_ERROR();
        return;
    }
    if( serial_number_num > 25 )
    {
        uart_Answer_ERRORn( ERROR_10H_F6_AT_NODE_OVER_FLOW );
        return;
    }
    tN1  = 1;
    tN2  = start_serial_number;                                                                                  //  tN2保存从节点起始序号
    for( i = 0; i < mCAC.i2AllNum; i++ )                                                         // 遍历从节点队列
    {
        if( uart_DAUType( i, 5 ) == TRUE1 )                                                     //  若是主动注册节点，tN1加1
        {
            if( tN1 >= tN2 )                                                                                  //  找到起始序号，跳出循环
            {
                break;
            }
            tN1++;
        }
    }
    tN2 = 0;
    pBuf[0] = '\r';
    // 组网节点个数
    NUM_2_ASCII_AT( serial_number_num, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    pBuf_index = temp_len1 + 2;
    for( ; i < mCAC.i2AllNum; i++ )                                                                //  继续遍历从节点序列
    {
        if( uart_DAUType( i, 5 ) == TRUE1 )                                                     //  若是主动注册节点，tN2加1
        {
            //   mDAU[i].aDAddr[0] =   mDAU[i].aDAddr[1] =   mDAU[i].aDAddr[2] =   mDAU[i].aDAddr[3] =   mDAU[i].aDAddr[4] =   mDAU[i].aDAddr[5] = 0x66;
            // 在网节点个数
            HEX_2_ASCII_AT( mDAU[i].aDAddr, pBuf + pBuf_index,  6 );
            pBuf_index += 12;
            pBuf[pBuf_index ++] =  ',';
            if( dauF_Good( i ) == TRUE1 )                                                                //  在网中继层次-1，不在网0x0F
            {
                relay_level = mDAU[i].b4DLayerSon - 1;                                            //  写中继级别，其余位默认全0
            }
            else
            {
                relay_level = 9;
            }
            pBuf[pBuf_index ++] = '0' + relay_level % 10 ;
            pBuf[pBuf_index ++] = ',';
            pBuf[pBuf_index ++] = '0' + ( INT8U )( mDAU[i].aDProType[1] );           // 通信协议类型 comm_type
            pBuf[pBuf_index ++] = ',';
            tN2++;
            if( tN2 >= serial_number_num || tN2 >= 25 )                                                    //  若tN2 达到25或要求查询数量，退出循环
            {
                break;
            }
        }
    }
    if( temp_len1 == 2 )
    {
        if( tN2 / 10 == 0 )
        {
            pBuf[1] = '0';
            pBuf[2] = '0' + tN2;
        }
        else
        {
            NUM_2_ASCII_AT( tN2, pBuf + 1,  &temp_len1 );
        }
    }
    else  if( temp_len1 == 1 )
    {
        pBuf[1] = '0' + tN2;
    }
    pBuf[--pBuf_index] = '\r';
    drv_UartSend( pBuf, pBuf_index + 1 );
}

/**
* @brief               查询网络规模
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN10_Fn100_AT_SSSL( INT8U* pBuf )
{
    INT8U len  = 0;
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( mCAC.i2Capacity, pBuf + 1, &len );
    pBuf[len + 1] = '\r';
    drv_UartSend( pBuf, len + 2 );
#ifdef  _LD_USE_TEST
    set_scan();
#endif
}

/**
* @brief              查询微功率无线从节点信息
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/09
*/
void DL2013_AFN10_Fn101_AT_NPWR( INT8U* pBuf )
{
    INT16U i, pBuf_index;                                                                    // 遍历索引（2B）
    INT16U tN2               =  0;                                                // 单帧个数计数
    INT16U li2MaxNum         =  mCAC.i2DownDAU;                                   // 档案总数
    INT16U li2LastNum        =  li2MaxNum;                                        // 剩余未上报总数
    INT8U temp_len1, temp_len, relay_level = 0, version_h, version_l, flag_send_once = 0;
    pBuf[0] = '\r';
    // 组网节点个数
    NUM_2_ASCII_AT( li2MaxNum, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    pBuf[temp_len1 + 2] = '0'; //本次传输的节点个数      -             十位
    pBuf[temp_len1 + 3] = '0'; //本次传输的节点个数      -             个位
    pBuf[temp_len1 + 4] = ',';
    pBuf_index = temp_len1 + 5;
    for( i = 0; i < mCAC.i2AllNum; i++ )                                          // 队列非空时，遍历队列
    {
        if( TRUE1 == uart_DAUType( i, 6 ) )                                         // 是微功率无线节点
        {
            // 在网节点个数
            HEX_2_ASCII_AT( mDAU[i].aDAddr, pBuf + pBuf_index,  6 );
            pBuf_index += 12;
            pBuf[pBuf_index ++] =  ',';
#if   PRO_AREA == 0                                                              // 国网标准: 上报中继级别
            relay_level      = ( TRUE1 == dauF_Good( i ) ) ? mDAU[i].b4DLayerSon - 1 : 0x0f;
#elif PRO_AREA == 1                                                              // 北京标准：上报层次 //目前按照这种
            relay_level      = ( TRUE1 == dauF_Good( i ) ) ? mDAU[i].b4DLayerSon - 1 : 9;
#endif
            pBuf[pBuf_index ++] = '0' + relay_level % 10 ;
            pBuf[pBuf_index ++] = ',';
            pBuf[pBuf_index ++] = '0' + ( INT8U )( mDAU[i].aDProType[1] );           // 通信协议类型 comm_type
            pBuf[pBuf_index ++] = ',';
            version_h      = ( TRUE1 == dauF_Good( i ) ) ? mDAU[i].aDVersion[0]    : 0;
            version_l      = ( TRUE1 == dauF_Good( i ) ) ? mDAU[i].aDVersion[1]    : 0;
            NUM_2_ASCII_AT( version_h, pBuf + pBuf_index, &temp_len );
            pBuf_index += temp_len;
            pBuf[pBuf_index ++] = '.';
            NUM_2_ASCII_AT( version_l, pBuf + pBuf_index, &temp_len );
            pBuf_index += temp_len;
            pBuf[pBuf_index ++] = ',';
            tN2++;                                                                    // 节点计数tN2加1
        }
        if( tN2 >= li2LastNum  ||  tN2 >= 20 )                                      // 若tN2满20或剩余总数，串口输出
        {
            pBuf[temp_len1 + 2] = tN2 / 10 + '0'; //本次传输的节点个数      -             十位
            pBuf[temp_len1 + 3] = tN2 % 10 + '0'; //本次传输的节点个数      -             个位
            pBuf[--pBuf_index] = '\r';
            drv_UartSend( pBuf, pBuf_index + 1 );
            pBuf_index = temp_len1 + 5;
            drv_stm32WDT();                                                           // 喂狗
            drv_Delay10ms( 50 );
            li2LastNum           =  li2LastNum - tN2;                                 // 更新剩余总数
            tN2                  =  0;                                                // 计数tN2清空
            flag_send_once = 1;
        }
        if( 0 == li2LastNum )                                                       // 剩余为0，可跳出遍历
        {
            break;
        }
    }
    if( 0 == mCAC.i2AllNum )                                                      // 队列为空时
    {
        drv_UartSend( "\r0\r", 3 );
        return;
    }
    if( flag_send_once == 0 )
    {
        uart_Answer_ERRORn( ERROR_10H_F101_AT_NO_THIS_NODE );
    }
}

/**
* @brief                 添加从节点
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN11_Fn1_NADD( INT8U* pBuf )
{
    INT16U i,  node_num, addr_index;
    INT8U addr_dau[6];
    if( pBuf[9] == ',' )
    {
        ASCII_2_NUM_AT( pBuf + 8, ( INT32U* )&node_num, 1 );
        addr_index = 10;
        if( pBuf[9 + 13 * node_num] != '\r' )
        {
            uart_Answer_ERROR();
            return;
        }
    }
    else if( pBuf[10] == ',' )
    {
        ASCII_2_NUM_AT( pBuf + 8, ( INT32U* )&node_num, 2 );
        addr_index = 11;
        if( pBuf[10 + 13 * node_num] != '\r' )
        {
            uart_Answer_ERROR();
            return;
        }
    }
    else
    {
        uart_Answer_ERROR();
        return;
    }
    if( node_num <= 20 )
    {
        for( i = 0; i < node_num; i++ )                                                              // 循环添加DAU
        {
            ASCII_2_HEX_AT( pBuf + addr_index + i * 13, addr_dau, 6 );
            // 添加addr
            cac_UserAddDAU( addr_dau );
        }
        cac_CountDAU();                                                                             // 统计更新DAU数量信息
        if( FALSE0 == mCAC.bSetup && FALSE0 == mCAC.bOptimize )                                     // 当前未组网且未维护
        {
            //   drv_Printf("\n本次下载%d个节点，其中有新增节点", pBuf[2]);
            drv_SetTimeDown( 40 );
        }
        uart_Answer_OK();                                                                          // 确认应答
    }
    else
    {
        uart_Answer_ERRORn( ERROR_11H_F1_AT_ADD_NUM_OVER_FLOW );
    }
}

/**
* @brief                  删除从节点
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN11_Fn2_NDEL( INT8U* pBuf )
{
    INT16U i, j, node_num, addr_index;
    INT8U addr_dau[6];  //flag_del_ok = 0;
    if( pBuf[9] == ',' )
    {
        ASCII_2_NUM_AT( pBuf + 8, ( INT32U* )&node_num, 1 );
        addr_index = 10;
        if( pBuf[9 + 13 * node_num] != '\r' )
        {
            uart_Answer_ERROR();
            return;
        }
    }
    else if( pBuf[10] == ',' )
    {
        ASCII_2_NUM_AT( pBuf + 8, ( INT32U* )&node_num, 2 );
        addr_index = 11;
        if( pBuf[10 + 13 * node_num] != '\r' )
        {
            uart_Answer_ERROR();
            return;
        }
    }
    else
    {
        uart_Answer_ERROR();
        return;
    }
    if( node_num <= 20 )
    {
        for( i = 0; i < node_num; i++ )                                                                          //  循环删除DAU
        {
            //flag_del_ok = 0;
            // 处理addr
            ASCII_2_HEX_AT( pBuf + addr_index + i * 13, addr_dau, 6 );
            for( j = 0; j < mCAC.i2AllNum; j++ )                                                     //  遍历定位待删除DAU在序列中位置
            {
                //执行 删除addr
                if( mDAU[j].bDUse == TRUE1
                        && mDAU[j].bDWrite == TRUE1
                        && Cb_CmpArry_stat( addr_dau, mDAU[j].aDAddr, 6 ) == TRUE1 )
                {
                    mDAU[j].bDUse = FALSE0;                                                               //  使用标志位标记为0
                    mDAU[j].bDWrite = FALSE0;                                                            //  下载标志位标记为0
                    mth_ClearRiss( j );                                                                            //  清空场强表
                    //   flag_del_ok = 1;
                    break;
                }
            }
            /*
             if(flag_del_ok == 0)
             {
                 uart_Answer_ERRORn(i + 1);
             }
             */
        }
        cac_CountDAU();                                                                                     //  统计更新DAU数量信息
        uart_Answer_OK();                                                                                   //  确认应答
    }
    else
    {
        uart_Answer_ERRORn( ERROR_11H_F2_AT_DEL_NUM_OVER_FLOW );
    }
}

/**
* @brief                  设置网络规模
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN11_Fn100_AT_SSSL( INT8U* pBuf )
{
    INT16U i, scale = 0;
    for( i = 0 ; i < 3; i++ )
    {
        if( pBuf[9 + i] == '\r' )
        {
            ASCII_2_NUM_AT( &( pBuf[8] ), ( INT32U* )&scale, i + 1 );
            if( ( scale < 2 ) || ( scale > 512 ) )
            {
                uart_Answer_ERRORn( ERROR_10H_F100_AT_SCALE_OVER_FLOW );
            }
            else
            {
                mCAC.i2Capacity = scale;
                cac_SaveCAC();
                uart_Answer_OK();                                                                                            // 返回确认
            }
            return;
        }
    }
    uart_Answer_Invalid_Command();
}


/**
* @brief                  启动网络维护进程
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN11_Fn101_AT_RTMT( INT8U* pBuf )
{
    // 若没有组网也没有网络维护
    if( FALSE0 == mCAC.bSetup && FALSE0 == mCAC.bOptimize )
    {
        // 倒数10s启动网络维护
        drv_SetTimeDown( 10 );
    }
    // 返回确认
    uart_Answer_OK();
}

/**
* @brief                 启动组网
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN11_Fn102_AT_RTMS( INT8U* pBuf )
{
    // 保存档案
    cac_SaveAll();
    // 若没有组网
    if( FALSE0 == mCAC.bSetup )
    {
        // 需要组网
        set_setup_son();
    }
    // 返回确认
    uart_Answer_OK();
}


/**
* @brief                 路由请求集中器时钟
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFN14_Fn2_AT_GCLK( INT8U* pBuf ) // 雷同与RTC
{
    INT8U  hour, minute, sec;
    ASCII_2_NUM_AT( pBuf + 8, ( INT32U* )&hour, 2 );
    if( hour > 24 )
    {
        uart_Answer_ERRORn( ERROR_05H_F100_AT_RSSIT_OVER_FLOW );
        return;
    }
    gSysTime.hour = hour;    // 设置小时
    ASCII_2_NUM_AT( pBuf + 10, ( INT32U* )&minute, 2 );
    if( minute > 60 )
    {
        uart_Answer_ERRORn( ERROR_05H_F100_AT_RSSIT_OVER_FLOW );
        return;
    }
    gSysTime.count = 60 * minute;
    ASCII_2_NUM_AT( pBuf + 12, ( INT32U* )&sec, 2 );
    if( sec > 60 )
    {
        uart_Answer_ERRORn( ERROR_05H_F100_AT_RSSIT_OVER_FLOW );
        return;
    }
    gSysTime.count += sec;                                             // 1h = 60min = 3600s
    if( ( gSysTime.hour == 0x21 ) && ( TskOptimizeLimitTime == 0 ) )
    {
        if( mCAC.bSetup != TRUE1 && nPrioIIFlag.fHourSate != 2 )                                // 有新增节点
        {
            nPrioIIFlag.fHourSate = 2;
            drv_SetTimeDown( 40 );
        }
    }
    uart_Answer_OK();
}


/**
* @brief                 读取场强
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
// 待续
void DL2013_AFN0A_Fn51_AT_REFI( INT8U* pBuf )
{
    INT16U tDAUN;
    INT16U i;
    INT8U k;
    INT8U tPage;
    INT8U* pRiss;
    tDAUN = pBuf[1];
    tDAUN = tDAUN << 8;
    tDAUN += pBuf[0];
    if( tDAUN == 0xFFFF )
    {
        pRiss = drvBuf.pRiss;
    }
    else
    {
        pRiss = drvBuf.pRiss + ( tDAUN + 1 ) * LEN_ONE_RISS;
    }
    tPage = 1;
    k = 0;
    for( i = 0; i <= MAX_DAU; i++ )
    {
        if( ( i & 1 ) == 0 )
        {
            pBuf[4 + k] = ( *pRiss ) >> 4;
        }
        else
        {
            pBuf[4 + k] = ( *pRiss ) & 0x0F;
            pRiss++;
        }
        k++;
        if( k >= 200 )
        {
            pBuf[2] = tPage;
            pBuf[3] = MAX_DAU / 200 + 1;
            // 返回数据
            uart_Answer_Data( 204 );
            // 再等待100ms
            drv_Delay10ms( 30 );
            k = 0;
            tPage++;
        }
    }
    // 最后一帧
    if( k > 0 )
    {
        pBuf[2] = tPage;
        pBuf[3] = MAX_DAU / 200 + 1;
        for( ; k < 200; k++ )
        {
            pBuf[4 + k] = 0;
        }
        // 返回数据
        uart_Answer_Data( 204 );
    }
}


/**
* @brief                 读取档案
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/09
*/
void DL2013_AFN0A_Fn52_AT_RAR( INT8U* pBuf )
{
    INT8U tN, temp_len1, temp_len2;
    INT16U i, pBuf_index = 0, j, flag_no_data = 0;
    INT16U tTop = 0xFFFF;
    INT16U tEnd = 0xFFFF;
    INT8U  addr_buf[512];
    tN = 0;
    for( i = 0; i < mCAC.i2AllNum; i++ )
    {
        if( tTop == 0xFFFF )
        {
            tTop = i;
        }
        tEnd = i;
        // 读取有效点
        if( dauF_Down( i ) == TRUE1 )
        {
            HEX_2_ASCII_AT( mDAU[i].aDAddr, addr_buf + pBuf_index,  6 );
            pBuf_index += 12;
            addr_buf[pBuf_index ++] =  ',';
        }
        else
        {
            for( j = 0; j < 12; j ++ )
            {
                addr_buf[pBuf_index ++] =  '0';
            }
            addr_buf[pBuf_index ++] =  ',';
        }
        tN++;
        if( tN >= 30 )
        {
            pBuf[0] = '\r';
            NUM_2_ASCII_AT( tTop, pBuf + 1,  &temp_len1 );
            pBuf[temp_len1 + 1] = ',';
            NUM_2_ASCII_AT( tEnd, pBuf + temp_len1 + 2,  &temp_len2 );
            pBuf[temp_len1 + temp_len2 + 2] = ',';
            addr_buf[pBuf_index - 1] =  '\r';
            mem_cpy( pBuf + temp_len1 + temp_len2 + 3, addr_buf, pBuf_index );
            drv_UartSend( pBuf, temp_len1 + temp_len2 + pBuf_index + 3 );
            // 再等待100ms
            drv_Delay10ms( 30 );
            tN = 0;
            flag_no_data = 1;
            tTop = 0xFFFF;
            memset( addr_buf, 0, sizeof( addr_buf ) );
            pBuf_index = 0;
        }
    }
    if( tN != 0 )
    {
        pBuf[0] = '\r';
        NUM_2_ASCII_AT( tTop, pBuf + 1,  &temp_len1 );
        pBuf[temp_len1 + 1] = ',';
        NUM_2_ASCII_AT( tEnd, pBuf + temp_len1 + 2,  &temp_len2 );
        pBuf[temp_len1 + temp_len2 + 2] = ',';
        addr_buf[pBuf_index - 1] =  '\r';
        mem_cpy( pBuf + temp_len1 + temp_len2 + 3, addr_buf, pBuf_index );
        drv_UartSend( pBuf, temp_len1 + temp_len2 + pBuf_index + 3 );
        flag_no_data = 1;
        // 再等待100ms
        drv_Delay10ms( 30 );
    }
    if( flag_no_data == 0 )
    {
        uart_Answer_ERROR();
    }
}

/**
* @brief                 读取CAC状态信息
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFNF0_Fn1_AT_CACS( INT8U* pBuf )
{
    INT8U temp_len1, temp_len2, temp_len3;
    pBuf[0] = '\r';
    if( mCAC.bSetup == TRUE1 )
    {
        pBuf[1] = '0';
    }
    else if( mCAC.bMeter == TRUE1 )
    {
        pBuf[1] = '1';
    }
    else
    {
        pBuf[1] = '2';
    }
    pBuf[2] = ',';
    cac_CountDAU();
    // 组网节点个数
    NUM_2_ASCII_AT( mCAC.i2InNetDAU, pBuf + 3,  &temp_len1 );
    pBuf[temp_len1 + 3] = ',';
    // 下载节点个数
    NUM_2_ASCII_AT( mCAC.i2GoodDAU, pBuf + temp_len1 + 4,  &temp_len2 );
    pBuf[temp_len1 + temp_len2 + 4] = ',';
    // 故障节点个数
    NUM_2_ASCII_AT( mCAC.i2BadDAU, pBuf + temp_len1 + temp_len2 + 5,  &temp_len3 );
    pBuf[temp_len1 + temp_len2 + temp_len3 + 5] = '\r';
    // 返回数据
    drv_UartSend( pBuf, temp_len1 + temp_len2 + temp_len3 + 6 );
}
/**
* @brief                CAC验证使用
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/

void DL2013_AFNF0_Fn100_AT_CACV( INT8U* pBuf )
{
    INT32U tCRC;
    INT8U CRC32_data[4];
    if( pBuf[16] != '\r' )
    {
        uart_Answer_ERROR();
        return;
    }
    ASCII_2_HEX_AT( pBuf + 8, CRC32_data, 4 );
    tCRC = LZCRC32( CRC32_data, 0, 4 );
    CRC32_data[0] = ( INT8U )( tCRC >> 0 );
    CRC32_data[1] = ( INT8U )( tCRC >> 8 );
    CRC32_data[2] = ( INT8U )( tCRC >> 16 );
    CRC32_data[3] = ( INT8U )( tCRC >> 24 );
    pBuf[0] = '\r';
    HEX_2_ASCII_AT( CRC32_data, pBuf + 1, 4 );
    pBuf[9] = '\r';
    drv_UartSend( pBuf,  10 );
}
/**
* @brief               读取内部版本号
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFNF0_Fn101_AT_RIV( INT8U* pBuf )
{
    // DT_PROTOCOL ,DT_EDITIOM_H.DT_EDITIOM_L,DT_CUSTOM
    INT8U temp_len1 = 0, temp_len2 = 0, temp_len3 = 0, temp_len4 = 0;
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( DT_PROTOCOL, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    NUM_2_ASCII_AT( DT_EDITIOM_H, pBuf + temp_len1 + 2,  &temp_len2 );
    pBuf[temp_len1 + temp_len2 + 2] = '.';
    NUM_2_ASCII_AT( DT_EDITIOM_L, pBuf + temp_len1 + temp_len2 + 3,  &temp_len3 );
    pBuf[temp_len1 + temp_len2 + temp_len3 + 3] = ',';
    NUM_2_ASCII_AT( DT_CUSTOM, pBuf + temp_len1 + temp_len2 + temp_len3 + 4,  &temp_len4 );
    pBuf[temp_len1 + temp_len2 + temp_len3 + temp_len4 + 4] = '\r';
    drv_UartSend( pBuf,  temp_len1 + temp_len2 + temp_len3 + temp_len4 + 5 );
}
/**
* @brief              读取CAC真实频道号
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFNF0_Fn102_AT_CACRC( INT8U* pBuf )
{
    INT8U len  = 0;
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( mCAC.i1BigCH, pBuf + 1, &len );
    pBuf[len + 1] = '\r';
    drv_UartSend( pBuf, len + 2 );
}







/**
* @brief           读取路径信息
                   0:读取0~4条路径
                   1：读取5~9条路径信息
                   2: 读取10~12条路径信息
                   3：读取当前使用路径信息
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/14
*/
void DL2013_AFNF0_Fn106_AT_RRPI( INT8U* pBuf )
{
    INT16U i;
    INT16U pNum = 0;
    INT8U tUseNum;
    INT8U n;
    INT8U dst_addr[6], path_channel, temp_len1, temp_len2;
    ASCII_2_HEX_AT( pBuf + 8, dst_addr, 6 );
    ASCII_2_NUM_U8_AT( pBuf + 21, &path_channel,   1 );
    if( path_channel > 6 )
    {
        uart_Answer_ERROR();
        return;
    }
    for( i = 0; i < MAX_DAU; i++ )
    {
        if( memcmp( mDAU[i].aDAddr, dst_addr, 6 ) == 0 )
        {
            pNum = i;    // 定位节点序号
            break;
        }
    }
    // 待续 01 14
    // 地址，{，，，{地址六组}}*2
    pBuf[0] = '\r';
    HEX_2_ASCII_AT( dst_addr, pBuf + 1, 6 );
    pBuf[13] = ',';
    if( path_channel != 6 )
    {
        mth_ReadmMath_AT( pBuf + 14, pNum, 2 * path_channel, &tUseNum, &temp_len1 );
        pBuf[14 + temp_len1] = ',';
        mth_ReadmMath_AT( pBuf + temp_len1 + 15, pNum, 2 * path_channel + 1, &tUseNum, &temp_len2 );
        pBuf[15 + temp_len1 + temp_len2] = '\r';
        drv_UartSend( pBuf, 16 + temp_len1  + temp_len2 );
    }
    else
    {
        mth_ReadmMath_AT( pBuf + 14, pNum, 12, &tUseNum, &temp_len1 );
        pBuf[14 + temp_len1] = '\r';
        drv_UartSend( pBuf, 15 + temp_len1 );
    }
}


/**
* @brief           掌机到集中器使用F0H 200
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/08
*/
void DL2013_AFNF0_Fn201_AT_TTTH( INT8U* pBuf )
{
    INT16U i, j, len;
    INT8U flag_get_data = 0;
    gDtHand* tHand = ( gDtHand* )( mRf.aBuf );
    mRf.i1Com   = RF_DEV_HAND;
    mRf.i1Layer = 1;
    tHand->i1Com = 0x34;
    tHand->i1Len = 0;
    for( i = 0 ; i < 3; i++ )
    {
        if( pBuf[9 + i] == ',' )
        {
            ASCII_2_NUM_AT( &( pBuf[8] ), ( INT32U* )&len, i + 1 );
            if( len > 255 )
            {
                uart_Answer_ERRORn( ERROR_F0H_F210_AT_LEN_OVER_FLOW );
                return;
            }
            else
            {
                tHand->i1Len = len;
                if( pBuf[10 + i + tHand->i1Len ] == '\r' )
                {
                    flag_get_data  = 1;
                    break;
                }
                else
                {
                    uart_Answer_ERROR();
                    return;
                }
            }
        }
    }
    if( flag_get_data  == 0 )
    {
        uart_Answer_ERROR();
        return;
    }
    for( j = 0; j < tHand->i1Len; j++ )
    {
        tHand->aBuf[j] = pBuf[1 + j + 9];
    }
    if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
    {
        drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
    }
    uart_Answer_OK();
}

/**
* @brief              查看串口接收的波特率当前参数
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/12
*/
void AT_IPR_INFO( INT8U* pBuf )
{
    pBuf[0] = '\r';
    pBuf[1] = '0' + AT_IPR_info ;
    pBuf[2] = '\r';
    drv_UartSend( pBuf, 3 );
}
/**
* @brief            设置串口接收的波特率
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/12
*/
void AT_IPR( INT8U* pBuf )
{
    if( pBuf[7] <= '6' )
    {
        uart_Answer_OK();
        sub_ax5043_delay( 80 );
    }
    switch( pBuf[7] )
    {
        case '1':
            sub_usart3_init( 4800, 8, 2 );
            break;
        case '2':
            sub_usart3_init( 9600, 8, 2 );
            break;
        case '3':
            sub_usart3_init( 14400, 8, 2 );
            break;
        case '4':
            sub_usart3_init( 19200, 8, 2 );
            break;
        case '5':
            sub_usart3_init( 38400, 8, 2 );
            break;
        case '6':
            sub_usart3_init( 57600, 8, 2 );
            break;
        default:
            uart_Answer_ERROR();
            return;
            break;
    }
    AT_IPR_info = pBuf[7] - '0';
}

/**
* @brief            检测DAU的状态信息 命令幀  点对点通讯
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/14
*/

void AT_W1( void ) // 检测DAU的状态信息 命令幀
{
    INT8U Buf[35] = {0x22,
                     0x43, 0xcd,
                     0x0f, 0xff, 0xff,
                     0x33, 0x63, 0x81, 0x52, 0x70, 0x11,
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
                     1, 2,
                     0x33, 0x63, 0x81, 0x52, 0x70, 0x11,
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
                     0, 0, 0
                    };
    drv_RfSend( Buf, CH_TYPE_TEST_0 );
    uart_Answer_OK();
}
/**
* @brief            检测DAU的状态信息 數據幀 点对点通讯
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/14
*/
void AT_W2( void ) // 检测DAU的状态信息 數據幀
{
    INT8U Buf[35] = {0x22,
                     0x41, 0xcd,
                     0x0f, 0xff, 0xff,
                     0x33, 0x63, 0x81, 0x52, 0x70, 0x11,
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
                     0x3c,
                     0x33, 0x63, 0x81, 0x52, 0x70, 0x11,
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
                     1, 1, 0, 4
                    };
    drv_RfSend( Buf, CH_TYPE_TEST_0 );
    uart_Answer_OK();
}

/**
* @brief            发送透传数据  点对点 數據幀
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/14
*/
void AT_W3( void ) // 发送透传数据  点对点 數據幀
{
    INT8U Buf[41] = {0x28,
                     0x41, 0xcd,
                     0x0f, 0xff, 0xff,
                     0x33, 0x63, 0x81, 0x52, 0x70, 0x11,
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
                     0x3c,
                     0x33, 0x63, 0x81, 0x52, 0x70, 0x11,
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
                     1, 1, 0, 0x31, 5, 'a', 'b', 'c', 'd', 'e'
                    };
    drv_RfSend( Buf, CH_TYPE_TEST_0 );
    uart_Answer_OK();
}




/**
* @brief            发送透传数据   兼容点对点及路由模式
* @param[in]  pBuf 接收和发送数据缓存数据指针
* @author       李晋南
* @date       2018/01/14
*/
void AT_TTD( INT8U* pBuf )
{
    INT8U Buf[35] = {0x22,
                     0x41, 0xcd,
                     0x0f,
                     0xff, 0xff, // panid
                     0x0, 0x0, 0x0, 0x0, 0x0, 0x0, // buf[6]
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
                     0x3c,
                     0x0, 0x0, 0x0, 0x0, 0x0, 0x0, //  buf[19]
                     0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
                     1, 1, 0, 0x31
                     // should add other , such as: datalen + data
                    };
    INT8U dst_addr[6], *temp_buf;
    INT8U i, flag_get_data = 0;
    INT16U len;
    if( *( pBuf + 19 ) != ',' )
    {
        uart_Answer_ERROR();
        return;
    }
    for( i = 0 ; i < 3; i++ )
    {
        if( pBuf[21 + i] == ',' )
        {
            ASCII_2_NUM_AT( &( pBuf[20] ), ( INT32U* )&len, i + 1 );
            if( len > 255 )
            {
                uart_Answer_ERROR();
                return;
            }
            if( *( pBuf + len + i + 22 ) != '\r' )
            {
                uart_Answer_ERROR();
                return;
            }
            flag_get_data = 1;
            break;
        }
    }
    if( flag_get_data == 0 )
    {
        uart_Answer_ERROR();
        return;
    }
    ASCII_2_HEX_AT( pBuf + 7, dst_addr, 6 );
    mem_cpy( Buf + 6, dst_addr, 6 );
    mem_cpy( Buf + 19, dst_addr, 6 );
    temp_buf = ( INT8U* )malloc( len );
    mem_cpy( temp_buf, pBuf + 22 + i, len );
    mem_cpy( pBuf, Buf, 0x23 );
    pBuf[0x23] = len;
    mem_cpy( pBuf + 0x24, temp_buf, len );
    pBuf[0] += ( len + 1 );
    drv_RfSend( pBuf, CH_TYPE_TEST_0 );
    free( temp_buf );
    uart_Answer_OK();
}




//-------------------------------------------------------------------------------------------------------------------------//




/**
* @brief  实现CFDA-AT的结构逻辑处理
* @param[in] pBuf   接收到串口数据的指针
* @param[in] pLen   接收到串口数据的长度
* @return
* @par 示例:
* @code
*        DL2013_AT_function（pBuf,100）;
* @endcode
* @deprecated
* @author       李晋南
* @date       2018/01/06
*/
void DL2013_AT_function( INT8U* pBuf, INT16U pLen )
{
    if( pBuf[3] == '/' )
    {
        mem_cpy( pBuf, AT_Save_Last_Cmd_Buf, AT_Save_Last_Cmd_Buf_Len );
        pLen =  AT_Save_Last_Cmd_Buf_Len ;
    }
    else
    {
        memset( AT_Save_Last_Cmd_Buf, 0, sizeof( AT_Save_Last_Cmd_Buf ) );
        AT_Save_Last_Cmd_Buf_Len = pLen;
        mem_cpy( AT_Save_Last_Cmd_Buf, pBuf, AT_Save_Last_Cmd_Buf_Len );
    }
    switch( pBuf[3] )
    {
        case 'A':
            if( memcmp( pBuf + 4, "REG=", 4 ) == 0 )
            {
                DL2013_AFN10_Fn6_AT_AREG( pBuf );
                return;
            }
            break;
        case 'B':
        {
            if( memcmp( pBuf + 4, "RDDT?\r", 6 ) == 0 )
            {
                DL2013_AFN03_Fn9_AT_BRDDT( pBuf );
                return;
            }
        }
        break;
        case 'C':
        {
            if( memcmp( pBuf + 4, "ACRC?\r", 6 ) == 0 )
            {
                DL2013_AFNF0_Fn102_AT_CACRC( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "ACS?", 4 ) == 0 )
            {
                DL2013_AFNF0_Fn1_AT_CACS( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "ACV=", 4 ) == 0 )
            {
                DL2013_AFNF0_Fn100_AT_CACV( pBuf );
                return;
            }
        }
        break;
        case 'D':
            break;
        case 'G':
            if( ( memcmp( pBuf + 4, "CLK=", 4 ) == 0 ) && ( pBuf[14] == '\r' ) )
            {
                //dai  DL2013_AFN14_Fn2_AT_GCLK(pBuf);
                // return;
            }
            break;
        case 'H':
        {
            switch( pBuf[4] )
            {
                case 'I':
                    if( memcmp( pBuf + 5, "NT\r", 3 ) == 0 )
                    {
                        DL2013_AFN01_Fn1_AT_HINT();
                        return;
                    }
                    break;
                case 'N':
                    if( memcmp( pBuf + 5, "A?\r", 3 ) == 0 )
                    {
                        DL2013_AFN03_Fn4_AT_HNA( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "A=", 2 ) == 0 ) && ( pBuf[19] == '\r' ) )
                    {
                        DL2013_AFN05_Fn1_AT_HNA( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "CP?\r", 4 ) == 0 ) )
                    {
                        DL2013_AFN03_Fn8_AT_HNCP( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "CP=", 3 ) == 0 ) )
                    {
                        DL2013_AFN05_Fn5_AT_HNCP( pBuf );
                        return;
                    }
                    break;
            }
        }
        break;
        case  'I':
            if( ( memcmp( pBuf + 4, "PR=", 3 ) == 0 ) && ( pBuf[8] == '\r' ) )
            {
                AT_IPR( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "PR?\r", 4 ) == 0 )
            {
                AT_IPR_INFO( pBuf );
                return;
            }
            break;
        case 'M':
        {
            if( memcmp( pBuf + 4, "VINFO?\r", 7 ) == 0 )
            {
                DL2013_AFN03_Fn1_AT_MVINFO( pBuf );
                return;
            }
        }
        break;
        case 'N':
        {
            if( memcmp( pBuf + 4, "NUM?\r", 5 ) == 0 )
            {
                DL2013_AFN10_Fn1_AT_NNUM( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "MMT?\r", 5 ) == 0 )
            {
                DL2013_AFN03_Fn7_AT_NMMT( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "MMT=", 4 ) == 0 )
            {
                DL2013_AFN05_Fn4_AT_NMMT( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "EREP=", 5 ) == 0 ) && ( pBuf[10] == '\r' ) )
            {
                DL2013_AFN05_Fn2_AT_NEREP( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "RC\r", 3 ) == 0 )
            {
                DL2013_AFN04_Fn2_AT_NRC();
                return;
            }
            else if( memcmp( pBuf + 4, "ADD=", 4 ) == 0 )
            {
                DL2013_AFN11_Fn1_NADD( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "DEL=", 4 ) == 0 )
            {
                DL2013_AFN11_Fn2_NDEL( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "INFO=", 5 ) == 0 )
            {
                DL2013_AFN10_Fn2_AT_NINFO( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "LST=", 4 ) == 0 )
            {
                DL2013_AFN10_Fn5_AT_NLST( pBuf );
                return;
            }
            else if( memcmp( pBuf + 4, "PWR?", 4 ) == 0 )
            {
                DL2013_AFN10_Fn101_AT_NPWR( pBuf );
                return;
            }
        }
        break;
        case 'P':
        {
            if( memcmp( pBuf + 4, "INT\r", 4 ) == 0 )
            {
                DL2013_AFN01_Fn2_F3_AT_PINT();
                return;
            }
        }
        break;
        case 'R':
        {
            switch( pBuf[4] )
            {
                case 'S':
                {
                    if( memcmp( pBuf + 5, "SIT?\r", 5 ) == 0 ) // 查询场强门限
                    {
                        DL2013_AFN03_Fn100_AT_RSSIT( pBuf );
                        return;
                    }
                    else  if( ( memcmp( pBuf + 5, "SIT=", 4 ) == 0 ) && ( ( pBuf[12] == '\r' ) || ( pBuf[11] == '\r' ) ) ) // 设置场强门限
                    {
                        DL2013_AFN05_Fn100_AT_RSSIT( pBuf );
                        return;
                    }
                }
                break;
                case 'I':
                {
                    if( memcmp( pBuf + 5, "V?\r", 3 ) == 0 ) // 查询内部版本号
                    {
                        DL2013_AFNF0_Fn101_AT_RIV( pBuf );
                        return;
                    }
                }
                break;
                case 'R':
                {
                    if( ( memcmp( pBuf + 5, "PI=", 3 ) == 0 ) && ( pBuf[20] == ',' ) && ( pBuf[22] == '\r' ) ) // 查询内部版本号
                    {
                        DL2013_AFNF0_Fn106_AT_RRPI( pBuf );
                        return;
                    }
                }
                break;
                case 'A':
                {
                    if( memcmp( pBuf + 5, "R?\r", 3 ) == 0 ) // 读取档案
                    {
                        DL2013_AFN0A_Fn52_AT_RAR( pBuf );
                        return;
                    }
                }
                break;
                case 'T':
                {
                    if( memcmp( pBuf + 5, "C?\r", 3 ) == 0 ) // 查询中心节点时间
                    {
                        DL2013_AFN03_Fn101_AT_RTC( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "C=", 2 ) == 0 ) && ( pBuf[19] == '\r' ) ) // 设置中心节点时间
                    {
                        DL2013_AFN05_Fn101_AT_RTC( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "MT\r", 3 ) == 0 ) ) // 启动维护
                    {
                        DL2013_AFN11_Fn101_AT_RTMT( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "RS\r", 3 ) == 0 ) ) // 启动组网
                    {
                        DL2013_AFN11_Fn102_AT_RTMS( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "ST?\r", 4 ) == 0 ) ) // 查询路由运行状态
                    {
                        DL2013_AFN10_Fn4_AT_RTST( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "MN=", 3 ) == 0 ) ) // 启动维护
                    {
                        DL2013_AFN03_Fn1_F101_AT_RTMN( pBuf );
                        return;
                    }
                }
                break;
            }
        }
        break;
        case 'S':
        {
            if( memcmp( pBuf + 4, "SCL?\r", 5 ) == 0 ) //查询网络规模
            {
                DL2013_AFN10_Fn100_AT_SSSL( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "SCL=", 4 ) == 0 ) )
            {
                DL2013_AFN11_Fn100_AT_SSSL( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "CCR?\r", 5 ) == 0 ) ) // 查看状态字和通信速率
            {
                DL2013_AFN03_Fn5_AT_SCCR( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "BRD=12,", 7 ) == 0 ) && ( pBuf[23] == '\r' ) ) // 广播校时
            {
                DL2013_AFN05_Fn3_AT_SBRD( pBuf );
                return;
            }
        }
        break;
        case 'T':
        {
            if( memcmp( pBuf + 4, "SST=", 4 ) == 0 ) // 发送测试命令
            {
                DL2013_AFN04_Fn1_AT_TSST( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "TTH=", 4 ) == 0 ) ) // 数据透传到掌机
            {
                DL2013_AFNF0_Fn201_AT_TTTH( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "TD=", 3 ) == 0 ) ) // 数据透传到掌机
            {
                AT_TTD( pBuf );
                return;
            }
        }
        break;
        case 'W':
            if( memcmp( pBuf + 4, "1\r", 2 ) == 0 ) //命令幀 点对点测试
            {
                AT_W1();
                return;
            }
            else  if( memcmp( pBuf + 4, "2\r", 2 ) == 0 )   //數據幀 点对点测试
            {
                AT_W2();
                return;
            }
            else  if( memcmp( pBuf + 4, "3\r", 2 ) == 0 ) //數據幀 点对点测试
            {
                AT_W3();
                return;
            }
            break;
        case '?':
        {
            if( memcmp( pBuf + 4, "\r", 1 ) == 0 )
            {
                uart_Answer_AT();
                return;
            }
        }
        break;
        default:
            break;
    }
    uart_Answer_Invalid_Command();
}
