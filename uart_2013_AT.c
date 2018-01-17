#include    "prio_II.h"
#include        "uart_2013_AT.h"

extern  INT8U   uartTxBuf[UART_ARG_1_LEN];          // ���崮�ڷ��ͻ�����
extern  INT8U   uart_path[36];
extern  INT8U   tempBuf22[256];
extern  INT8U   backMeter;
extern INT16U TskOptimizeLimitTime;
extern INT8U        gFlgUrtCmpFile;                         // ����ͬ�����̱�־
extern INT8U        FlagNew;                                                    // �ϱ���־λ�����ڼ�������ע��

INT16U      xb_Capacity( INT16U pN );
INT8U   AT_Save_Last_Cmd_Buf[LEN_RF], AT_Save_Last_Cmd_Buf_Len = 0;
INT8U   AT_IPR_info = 2; // ��������ֵ



const INT8U MON[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
INT64U time_total_from_1970 = 0;



/**
* @brief  ʵ�ּ򵥵�10�Ĵη�����
* @param[in] pown    10��pown�η�
* @return ������ȡ�Ĵη����
* @par ʾ��:
* @code
*        int num = pow_10_n(10);
* @endcode
* @deprecated ע��ƽ����Ŀ��Ҫ����INT32U�ķ�Χ
* @author       �����
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
* @brief  ASCII����ת��Ϊ���ָ�ʽ
* @param[in] pBuf       ASCII��ָ����׵�ַ
* @param[in] num        ��Ҫ��õ����ֵĵ�ַ
* @param[in] len        ��Ҫ��ȡ�жϵ�ASCII������ݳ��ȣ���pBuf��ַ��ʼ�������
* @return
* @par ʾ��:
* @code
*        ASCII_2_NUM_AT(pBuf, &num, 2);
* @endcode
* @deprecated ÿһλASCII��ķ�Χ����'0' ~ ��9��
* @author       �����
* @date       2018/01/08
*/

void ASCII_2_NUM_AT( INT8U* pBuf, INT32U* num, INT8U len ) // len ΪASCII����
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

void ASCII_2_NUM_U8_AT( INT8U* pBuf, INT8U* num, INT8U len ) // len ΪASCII����
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
* @brief  ASCII����ת��Ϊ���ָ�ʽ,INT16U ����
* @param[in] pBuf       ASCII��ָ����׵�ַ
* @param[in] num        ��Ҫ��õ����ֵĵ�ַ
* @param[in] len        ��Ҫ��ȡ�жϵ�ASCII������ݳ��ȣ���pBuf��ַ��ʼ�������
* @return
* @par ʾ��:
* @code
*        ASCII_2_NUM_AT(pBuf, &num, 2);
* @endcode
* @deprecated ÿһλASCII��ķ�Χ����'0' ~ ��9��
* @author       �����
* @date       2018/01/08
*/

void ASCII_2_NUM_U16_AT( INT8U* pBuf, INT16U* num, INT8U len ) // len ΪASCII����
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
* @brief  ���ָ�ʽת��ΪASCII����
* @param[in] num        ת�������ֵ�ֵ
* @param[in] pBuf       ת���õ���ASCII��ָ���ַ
* @param[in] len        ת����õ�ASCII��ĳ��ȴ�С��ָ��
* @return
* @par ʾ��:
* @code
*        ASCII_2_NUM_AT(127, buf, &len);
* @endcode
* @deprecated ���ݳ��ȷ�Χ��Ҫ����INT32U��Χ��ָ�볤�ȷ�ֹ�����쳣�δ���
* @author       �����
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
* @brief  HEX��ʽ����ת��ΪASCII�����ݣ����� 0x6800 ת��Ϊ ��6800��
* @param[in] pBufhex        ��Ҫת����hex�����׵�ַ
* @param[in] pBuf           ת���õ���ASCII��ָ���ַ
* @param[in] hex_len        ת��hex����ĳ���
* @return
* @par ʾ��:
* @code
*        HEX_2_ASCII_AT(pBufhex, buf, len);
* @endcode
* @deprecated
* @author       �����
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
* @brief  ASCII������ת��ΪHEX��ʽ���ݣ����� ��6800�� ת��Ϊ 0x6800
* @param[in] pBuf          ��Ҫת����ASCII�����׵�ַ
* @param[in] pBufhex       ת���õ���HEXָ���ַ
* @param[in] hex_len       ת��Ϊhex����ĳ���
* @return
* @par ʾ��:
* @code
*        ASCII_2_HEX_AT(pBufhex, buf, len);
* @endcode
* @deprecated
* @author       �����
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
    tTime->hour = ( lSec / 3600 ) % 24; //����ע�⣬����ʱ���Ѿ����ϱ���ʱ���8��
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
    drv_UartSend( "\rAT:1\r", 5 ); // AT: 1  1 ATָ��汾��
}
/**
* @brief   Ӳ����ʼ����CAC��λ
* @param[in]  ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN01_Fn1_AT_HINT( void )
{
    uart_Answer_OK();
    drv_Delay10ms( 20 );                                                                       //  �ȴ��������
    cac_SaveAll();
    drv_Resetcac();                                                                                  //  CAC��λ
}


/**
* @brief   ������ʼ�����������DAU����
* @param[in]  ��
* @author       �����
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
* @brief   ��������Ϣ
* @param[in]  ��
* @author       �����
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
    //  ���г�������
    if( mCAC.bMeter == TRUE1 )
    {
        //drv_Printf("\n���г�������");
        free( Get645_data );
        uart_Answer_ERROR(); // ��Ķ�Ϊ����������
        return ;                                                                //  �������ڵ�æ
    }
    //  ����DAU���
    tmpDAUNum = cac_CheckDAUNum( Dau_Addr );
    //  drv_Printf( "\nĿ��DAU[%3d] =  ", tmpDAUNum );
    drv_PrintfDAU( pDAU );
    //  �ڵ㲻�ڵ���
    if( tmpDAUNum >= MAX_DAU )
    {
        // drv_Printf("\n�ڵ㲻�ڵ���");
        free( Get645_data );
        uart_Answer_ERROR(); // ��Ķ�Ϊ����������  //  ���ز�����
        return ;
    }
    //  ��Ϊ͸������
    if( cmd_type == 0x00 )
    {
        //  ����DAUЭ������
        mDAU[tmpDAUNum].aDProType[0] = Get645FrameType( Get645_data );
    }
    //  ����Э���ֶ�
    else
    {
        //  ��������֡����
        tmpFrameType = Get645FrameType( Get645_data );
        mDAU[tmpDAUNum].aDProType[0] = tmpFrameType;
    }
    // drv_Printf("\n==============================��������=============================");
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
* @brief    ��ѯ���̴���Ͱ汾��Ϣ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief    ��ѯ���ڵ��ַ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN03_Fn4_AT_HNA( INT8U* pBuf )
{
    pBuf[0] = '\r';
    //  дCAC��ַ�����ݵ�Ԫ
    HEX_2_ASCII_AT( mCAC.aCAddr, pBuf + 1, 6 );
    pBuf[13] = '\r';
    drv_UartSend( pBuf, 14 );
}

/**
* @brief    ��ѯ���ڵ�״̬�ֺ�ͨ������
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN03_Fn5_AT_SCCR( INT8U* pBuf )
{
    INT8U  status_world[2], temp_len1;
    //  ���ڵ�״̬��
    // ���ڳ���ģʽ=01b                          01   ������������(����); 10   ��·������(�ز�) ; 11 ���ֶ�֧��(˫ģ�칹)
    //  ���ڵ��ŵ����� =00b                 00 ΢�������ߣ�01 ���๩�絥�ഫ�䣻10 ����������11��������
    //  ͨ���������� =  1��0001b��
    //  pBuf[1] ����λ���ã�����λ�ŵ�����
    status_world[0] = 0x41; //
    status_world[1] = 0x1f; //  �ŵ�ʵ������32
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( status_world[1], pBuf + 1, &temp_len1 ); //�ŵ�ʵ������
    pBuf[temp_len1 + 1] = '\r';
    // ��������
    drv_UartSend( pBuf, temp_len1 + 2 );
}

/**
* @brief      ��ȡ�ӽڵ������ʱʱ��
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN03_Fn7_AT_NMMT( INT8U* pBuf )
{
    INT8U len = 0;
    pBuf[0] = '\r';
    //  дCAC��ַ�����ݵ�Ԫ
    NUM_2_ASCII_AT( mCAC.i1ReadMeterMaxTime, pBuf + 1, &len );
    pBuf[len + 1] = '\r';
    drv_UartSend( pBuf, len + 2 );
}


/**
* @brief      ��ѯ����ͨ�Ų���
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief       ��ѯͨ����ʱ��صĹ㲥ʱ��
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN03_Fn9_AT_BRDDT( INT8U* pBuf )
{
    INT8U temp_len1 = 0;
    INT16U DelayTime = cac_GetBroadDelayTime( mCAC.i2AllNum + 5 ); //cac_GetBroadDelayTime(mCAC.i2ShiXi);//
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( DelayTime, pBuf + 1,  &temp_len1 ); //  �㲥��ʱ����ʱ϶����
    pBuf[temp_len1 + 1] = '\r';
    drv_UartSend( pBuf, temp_len1 + 2);
}


/*
INT8U Data03HF10_(INT8U *xBuf)
{
    INT8U i;

    // ���ڳ���ģʽ=01 b                          01   ������������(����); 10   ��·������(�ز�) ; 11 ���ֶ�֧��(˫ģ�칹)
    // �ӽڵ���Ϣģʽ=1 b                       ��Ҫ�·��ӽڵ���Ϣ
    // ·�ɹ���ʽ=1 b                            ����ģ����·�ɹ���
    // ͨ�ŷ�ʽ=0011 b                                 1  խ���ز�; 2  ����ز�; 3  ����΢����; ����  ����
    xBuf[0] = 0x73;                                                                                          //  01 1 1 0011

    // �㲥�����ŵ�ִ�з�ʽ=00 b     �㲥������ŵ���ʶ
    // �㲥����ȷ�Ϸ�ʽ=1 b                  �㲥���ǰ����ȷ�ϱ��ģ�ȷ�ϱ�����''�ȴ�ִ��ʱ�䡱��Ч��
    // ʧ�ܽڵ��л�����ʽ=10 b     10 ����������; 01  ����ģ�������л�
    // ������ʱ����֧��=110 b             ���α�ʾ�㲥���ӽڵ��ء�·�����������������ṩ����
    //                                                                      ʱ�Ӳ����������1��֧��
    xBuf[1]=0x36;

    // ��ѹ����������Ϣ= 000b            ���δ���A, B, C���������Ϣ��0��δ����
    // �ŵ�����= 31(���ֵ)
    xBuf[2]=0x1f;

    //  ��4λ����
    //  ��4λ ��������=1
    xBuf[3] = 0x01;

    // �����ֶΣ�д0
    xBuf[4]=0;
    xBuf[5]=0;

    // �ӽڵ������ʱʱ��
    xBuf[6]     =   mCAC.i1ReadMeterMaxTime;

    // �㲥�������ʱʱ��
    xBuf[7]     =   (INT8U)TIME_MAX_BROADCAST;
    xBuf[8]     =   (INT8U)(TIME_MAX_BROADCAST >> 8);

    // ���֧�ֱ��ĳ���
    xBuf[9]     =   255;
    xBuf[10]    =   0;

    // �ļ�����֧�ֵ���󵥸����ݰ�����
    xBuf[11]    =   128;
    xBuf[12]    =   0;

    // ���������ȴ�ʱ��
    xBuf[13]    =   TIME_MAX_UPDATE;

    // ���ڵ��ַ
    xBuf[14]=mCAC.aCAddr[0];
    xBuf[15]=mCAC.aCAddr[1];
    xBuf[16]=mCAC.aCAddr[2];
    xBuf[17]=mCAC.aCAddr[3];
    xBuf[18]=mCAC.aCAddr[4];
    xBuf[19]=mCAC.aCAddr[5];

    // ֧�����ӽڵ�����
    xBuf[20]=(INT8U)MAX_DAU;
    xBuf[21]=(INT8U)(MAX_DAU>>8);

    // ��ǰ�ӽڵ�����
    xBuf[22]=(INT8U)mCAC.i2GoodDAU;
    xBuf[23]=(INT8U)(mCAC.i2GoodDAU>>8);

    // ͨ��ģ��ʹ�õ�Э�鷢������
    xBuf[24]=0x21;
    xBuf[25]=0x03;
    xBuf[26]=0x13;

    // ͨ��ģ��ʹ�õ�Э�鱸�ݷ�������
    xBuf[27]=0x21;
    xBuf[28]=0x03;
    xBuf[29]=0x13;

    // ������Ϣ��ʾ
    xBuf[30]='C';
    xBuf[31]='F';
    xBuf[32]='M';
    xBuf[33]='J';
    xBuf[34]=DT_DATA;
    xBuf[35]=DT_MOTH;
    xBuf[36]=DT_YEAR;
    xBuf[37]=DT_EDITIOM_L;
    xBuf[38]=DT_EDITIOM_H;

    //  ͨ����������Ϣ
    //  D15: ���ʵ�λ��ʾ; 1��ʾKbps��0��ʾbps
    //  D14~D0 ͨ������=0Ĭ�ϣ�
    for(i=0;i<xBuf[3];i++)
    {
        xBuf[39+i*2]=0;
        xBuf[40+i*2]=0x80;
    }

    return (xBuf[3] *2 + 39);
}
*/

/**
* @brief           ��ѯ����ͨ��ģ������ģʽ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN03_Fn10_AT_HMOD( INT8U* pBuf )  //����
{
    INT8U length = 0;
    //length = Data03HF10(pBuf);                                                                  //  д�û����ݵ�Ԫ
    drv_UartSend( pBuf, length );
}



/**
* @brief         ��ѯ��ǿ����
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN03_Fn100_AT_RSSIT( INT8U* pBuf )
{
    //  ��ǿ���ޣ�ȡֵ50~120
    INT8U len  = 0;
    pBuf[0] = '\r';
    NUM_2_ASCII_AT( mCAC.i1Valve, pBuf + 1, &len );
    pBuf[len + 1] = '\r';
    drv_UartSend( pBuf, len + 2 );
}


/**
* @brief         ��ѯ���Ľڵ�ʱ��
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief         ��·�ӿڷ��Ͳ���
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief        ��·�ӿڴӽڵ����
* @param[in]
* @author       �����
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
* @brief         �������ڵ��ַ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN05_Fn1_AT_HNA( INT8U* pBuf )
{
    INT8U i = 0, addr_buf[6];
    ASCII_2_HEX_AT( pBuf + 7, addr_buf, 6 );
    // �����ڵ��ַ�䶯
    if( memcmp( mCAC.aCAddr, addr_buf, 6 ) != 0 )
    {
        // ��ȡCAC��ַ
        for( i = 0; i < 6; i++ )
        {
            mCAC.aCAddr[i] = addr_buf[i] ;
        }
        // ����panID
        LongToPanID( mCAC.aCAddr, mCAC.panID );
        // ����ʵ�ʹ���Ƶ��
        mCAC.i1BigCH = ( INT8U )( ( mCAC.panID[0] % ( SUM_CH_NUM - 1 ) ) + 1 );
        drv_setCH( mCAC.i1BigCH );
        // ��Ҫ���ݵ����Զ�����Ƶ��
        mCAC.bNewWrkCH  =   TRUE1;
        // ����CAC�����Ϣ
        cac_SaveCAC();
    }
    uart_Answer_OK();
}

/**
* @brief         ����/��ֹ�ӽڵ��ϱ�
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
        mCAC.bReport = TRUE1;    //  �����ϱ�
    }
    else if( pBuf[9] == '0' )
    {
        mCAC.bReport = FALSE0;    //  ��ֹ�ϱ�
    }
    uart_Answer_OK();                                                                                       // ����ȷ��
}

/**
* @brief         �����㲥
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08 ���� & 2018/01/15 ����
*/
void DL2013_AFN05_Fn3_AT_SBRD( INT8U* pBuf )
{
    gDtMeter* tM = ( gDtMeter* )( mRf.aBuf );
    INT8U year, month, day, hour, minute, sec, i;
    INT8U conformation_buf[20] = {0x01, 0x12,
                                  0x68, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x68, 0x08,
                                  0x06, //    buf[11] ����
                                  0x37, 0x53, 0x44, 0x48, 0x34, 0x4B, //   buf[12] �� �� ʱ�� �� �� �� +33
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
    rf_BroadcastTime( &( conformation_buf[2] ), conformation_buf[1] );                                 //  �㲥Уʱ
    drv_SetTimeMeter( ( INT16U )( xb_Capacity( mCAC.i2DownDAU ) ) );             //  ��̬��ģ
    mCAC.bMeter = TRUE1;
    mCAC.bBroad = TRUE1;
    uart_Answer_OK();// ����ȷ��
}


/**
* @brief          ���ôӽڵ������ʱʱ��
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
            uart_Answer_OK();                                                                                            // ����ȷ��
            return;
        }
    }
    uart_Answer_Invalid_Command();
}

/**
* @brief          ��������ͨ�Ų���
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
            // ����ֻ���ĵ�
            if( Power <= 4 )
            {
                mCAC.i1RFPower = Power;
            }
            else
            {
                uart_Answer_ERRORn( ERROR_05H_F5_AT_POWER_OVER_FLOW );
            }
            // �Զ�ѡ���Ĭ�ϣ����ı�
            if( speed >= 254 )                                                                              //  254�Զ�ѡ��
            {
                uart_Answer_OK();
            }
            else if( ( speed < SUM_CH_NUM ) && ( speed > 0 ) )                                            //  1~32     // 1 ~ 32�����±���Ƶ��
            {
                mCAC.i1BigCHXX = speed;
                cac_SaveCAC();
                uart_Answer_OK();
            }
            else                                                                                                 //  ����
            {
                uart_Answer_ERRORn( ERROR_05H_F5_AT_SPEED_OVER_FLOW );
            }
            return;
        }
    }
    uart_Answer_Invalid_Command();
}


/**
* @brief           ���ó�ǿ����
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
        uart_Answer_OK();                                                                                       // ����ȷ��
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
    //��1 3 5 7 8 10 12
    //С��4 6 9 11
    //ƽ��2
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
* @brief             �������Ľڵ�ʱ��
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
    gSysTime.hour = hour;    // ����Сʱ
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
        if( mCAC.bSetup != TRUE1 && nPrioIIFlag.fHourSate != 2 )                                                             // �������ڵ�
        {
            nPrioIIFlag.fHourSate = 2;
            drv_SetTimeDown( 10 );
        }
    }
    time_total_from_1970 = mktime( year, month, day, hour, minute, sec );
    uart_Answer_OK();
}



/**
* @brief              ��ѯ�ӽڵ�����
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief      ��ѯ�ӽڵ���Ϣ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
    // �����ڵ����
    NUM_2_ASCII_AT( serial_number_num, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    pBuf_index = temp_len1 + 2;
    tN1  = 1;
    tN2  = start_serial_number;                                                                                //  tN2����ӽڵ���ʼ���
    for( i = 0; i < mCAC.i2AllNum; i++ )                                                         // �����ӽڵ����
    {
        if( uart_DAUType( i, 3 ) == TRUE1 )                                                     //  �������ؽڵ㣬tN1��1
        {
            if( tN1 >= tN2 )                                                                                  //  �ҵ���ʼ��ţ�����ѭ��
            {
                break;
            }
            tN1++;
        }
    }
    tN2 = 0;
    for( ; i < mCAC.i2AllNum; i++ )                                                                //  ���������ӽڵ�����
    {
        if( uart_DAUType( i, 3 ) == TRUE1 )                                                     //  �������ؽڵ㣬tN2��1
        {
            HEX_2_ASCII_AT( mDAU[i].aDAddr, pBuf + pBuf_index,  6 );
            pBuf_index += 12;
            pBuf[pBuf_index ++] =  ',';
            if( dauF_Good( i ) == TRUE1 )                                                        //  830̨��ӿ�:  0~6���м̲�Σ�0x0f������
            {
                relay_level = mDAU[i].b4DLayerSon - 1;
            }
            else
            {
                relay_level = 9;
            }
            pBuf[pBuf_index ++] = '0' + relay_level % 10 ;
            pBuf[pBuf_index ++] = ',';
            pBuf[pBuf_index ++] = '0' + ( INT8U )( mDAU[i].aDProType[1] );           // ͨ��Э������ comm_type
            pBuf[pBuf_index ++] = ',';                                                    // Э������:  ��������
            tN2++;
            if( tN2 >= serial_number_num || tN2 >= 25 )                                                   // ��tN2 �ﵽ25��Ҫ���ѯ�������˳�ѭ��
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
    if( FALSE0 == mCAC.bSetup && FALSE0 == mCAC.bOptimize && TRUE1  == gFlgUrtCmpFile )         // CACδ������δά���ҿ�ʼ����ͬ��
    {
        //  drv_Printf("\n��������%d���ڵ㣬�����������ڵ�", pBuf[2]);
        drv_SetTimeDown( 40 );
    }
}

/**
* @brief               ��ѯ·������״̬
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN10_Fn4_AT_RTST( INT8U* pBuf )
{
    INT8U temp_len1, temp_len2, work_switch = 0;
    pBuf[0] = '\r';
    // �����ڵ����
    NUM_2_ASCII_AT( mCAC.i2DownDAU, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    // �����ڵ����
    NUM_2_ASCII_AT( mCAC.i2GoodDAU, pBuf + temp_len1 + 2,  &temp_len2 );
    pBuf[temp_len1 + temp_len2 + 2] = ',';
    //  д�������ظ�ʽ
    if( mCAC.bYuLi == TRUE1 )                                                                               //  �ѱ�
    {
        work_switch += 2;
    }
    if( mCAC.bSetup == TRUE1 )                                                                            //  ����
    {
        work_switch += 1;
    }
    pBuf[temp_len1 + temp_len2  + 3] = work_switch +  '0';
    pBuf[temp_len1 + temp_len2 + 4] = '\r';
    // ��������
    drv_UartSend( pBuf, temp_len1 + temp_len2 + 5 );
}



/**
* @brief         ��ѯδ�����ɹ��ӽڵ���Ϣ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
    tN2  = start_serial_number;                                                                                  //  tN2����ӽڵ���ʼ���
    for( i = 0; i < mCAC.i2AllNum; i++ )                                                         // �����ӽڵ����
    {
        if( uart_DAUType( i, 2 ) == TRUE1 )                                                     //  ���ǲ������ڵ㣬tN1��1
        {
            if( tN1 >= tN2 )                                                                                  //  �ҵ���ʼ��ţ�����ѭ��
            {
                break;
            }
            tN1++;
        }
    }
    tN2 = 0;
    pBuf[0] = '\r';
    // �����ڵ����
    NUM_2_ASCII_AT( serial_number_num, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    pBuf_index = temp_len1 + 2;
    for( ; i < mCAC.i2AllNum; i++ )                                                                //  ���������ӽڵ�����
    {
        if( uart_DAUType( i, 2 ) == TRUE1 )                                                     //  ���ǲ����������Ͻڵ㣬tN2��1
        {
            //   mDAU[i].aDAddr[0] =   mDAU[i].aDAddr[1] =   mDAU[i].aDAddr[2] =   mDAU[i].aDAddr[3] =   mDAU[i].aDAddr[4] =   mDAU[i].aDAddr[5] = 0x66;
            HEX_2_ASCII_AT( mDAU[i].aDAddr, pBuf + pBuf_index,  6 );
            pBuf_index += 12;
            pBuf[pBuf_index ++] =  ',';
            tN2++;
            if( tN2 >= serial_number_num || tN2 >= 25 )                                                       //  ��tN2 �ﵽ25��Ҫ���ѯ�������˳�ѭ��
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
* @brief       ��ѯ����ע��ӽڵ���Ϣ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN10_Fn6_AT_AREG( INT8U* pBuf ) // 0108����֤����ʵ�ʲ��Է�������
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
    tN2  = start_serial_number;                                                                                  //  tN2����ӽڵ���ʼ���
    for( i = 0; i < mCAC.i2AllNum; i++ )                                                         // �����ӽڵ����
    {
        if( uart_DAUType( i, 5 ) == TRUE1 )                                                     //  ��������ע��ڵ㣬tN1��1
        {
            if( tN1 >= tN2 )                                                                                  //  �ҵ���ʼ��ţ�����ѭ��
            {
                break;
            }
            tN1++;
        }
    }
    tN2 = 0;
    pBuf[0] = '\r';
    // �����ڵ����
    NUM_2_ASCII_AT( serial_number_num, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    pBuf_index = temp_len1 + 2;
    for( ; i < mCAC.i2AllNum; i++ )                                                                //  ���������ӽڵ�����
    {
        if( uart_DAUType( i, 5 ) == TRUE1 )                                                     //  ��������ע��ڵ㣬tN2��1
        {
            //   mDAU[i].aDAddr[0] =   mDAU[i].aDAddr[1] =   mDAU[i].aDAddr[2] =   mDAU[i].aDAddr[3] =   mDAU[i].aDAddr[4] =   mDAU[i].aDAddr[5] = 0x66;
            // �����ڵ����
            HEX_2_ASCII_AT( mDAU[i].aDAddr, pBuf + pBuf_index,  6 );
            pBuf_index += 12;
            pBuf[pBuf_index ++] =  ',';
            if( dauF_Good( i ) == TRUE1 )                                                                //  �����м̲��-1��������0x0F
            {
                relay_level = mDAU[i].b4DLayerSon - 1;                                            //  д�м̼�������λĬ��ȫ0
            }
            else
            {
                relay_level = 9;
            }
            pBuf[pBuf_index ++] = '0' + relay_level % 10 ;
            pBuf[pBuf_index ++] = ',';
            pBuf[pBuf_index ++] = '0' + ( INT8U )( mDAU[i].aDProType[1] );           // ͨ��Э������ comm_type
            pBuf[pBuf_index ++] = ',';
            tN2++;
            if( tN2 >= serial_number_num || tN2 >= 25 )                                                    //  ��tN2 �ﵽ25��Ҫ���ѯ�������˳�ѭ��
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
* @brief               ��ѯ�����ģ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief              ��ѯ΢�������ߴӽڵ���Ϣ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/09
*/
void DL2013_AFN10_Fn101_AT_NPWR( INT8U* pBuf )
{
    INT16U i, pBuf_index;                                                                    // ����������2B��
    INT16U tN2               =  0;                                                // ��֡��������
    INT16U li2MaxNum         =  mCAC.i2DownDAU;                                   // ��������
    INT16U li2LastNum        =  li2MaxNum;                                        // ʣ��δ�ϱ�����
    INT8U temp_len1, temp_len, relay_level = 0, version_h, version_l, flag_send_once = 0;
    pBuf[0] = '\r';
    // �����ڵ����
    NUM_2_ASCII_AT( li2MaxNum, pBuf + 1,  &temp_len1 );
    pBuf[temp_len1 + 1] = ',';
    pBuf[temp_len1 + 2] = '0'; //���δ���Ľڵ����      -             ʮλ
    pBuf[temp_len1 + 3] = '0'; //���δ���Ľڵ����      -             ��λ
    pBuf[temp_len1 + 4] = ',';
    pBuf_index = temp_len1 + 5;
    for( i = 0; i < mCAC.i2AllNum; i++ )                                          // ���зǿ�ʱ����������
    {
        if( TRUE1 == uart_DAUType( i, 6 ) )                                         // ��΢�������߽ڵ�
        {
            // �����ڵ����
            HEX_2_ASCII_AT( mDAU[i].aDAddr, pBuf + pBuf_index,  6 );
            pBuf_index += 12;
            pBuf[pBuf_index ++] =  ',';
#if   PRO_AREA == 0                                                              // ������׼: �ϱ��м̼���
            relay_level      = ( TRUE1 == dauF_Good( i ) ) ? mDAU[i].b4DLayerSon - 1 : 0x0f;
#elif PRO_AREA == 1                                                              // ������׼���ϱ���� //Ŀǰ��������
            relay_level      = ( TRUE1 == dauF_Good( i ) ) ? mDAU[i].b4DLayerSon - 1 : 9;
#endif
            pBuf[pBuf_index ++] = '0' + relay_level % 10 ;
            pBuf[pBuf_index ++] = ',';
            pBuf[pBuf_index ++] = '0' + ( INT8U )( mDAU[i].aDProType[1] );           // ͨ��Э������ comm_type
            pBuf[pBuf_index ++] = ',';
            version_h      = ( TRUE1 == dauF_Good( i ) ) ? mDAU[i].aDVersion[0]    : 0;
            version_l      = ( TRUE1 == dauF_Good( i ) ) ? mDAU[i].aDVersion[1]    : 0;
            NUM_2_ASCII_AT( version_h, pBuf + pBuf_index, &temp_len );
            pBuf_index += temp_len;
            pBuf[pBuf_index ++] = '.';
            NUM_2_ASCII_AT( version_l, pBuf + pBuf_index, &temp_len );
            pBuf_index += temp_len;
            pBuf[pBuf_index ++] = ',';
            tN2++;                                                                    // �ڵ����tN2��1
        }
        if( tN2 >= li2LastNum  ||  tN2 >= 20 )                                      // ��tN2��20��ʣ���������������
        {
            pBuf[temp_len1 + 2] = tN2 / 10 + '0'; //���δ���Ľڵ����      -             ʮλ
            pBuf[temp_len1 + 3] = tN2 % 10 + '0'; //���δ���Ľڵ����      -             ��λ
            pBuf[--pBuf_index] = '\r';
            drv_UartSend( pBuf, pBuf_index + 1 );
            pBuf_index = temp_len1 + 5;
            drv_stm32WDT();                                                           // ι��
            drv_Delay10ms( 50 );
            li2LastNum           =  li2LastNum - tN2;                                 // ����ʣ������
            tN2                  =  0;                                                // ����tN2���
            flag_send_once = 1;
        }
        if( 0 == li2LastNum )                                                       // ʣ��Ϊ0������������
        {
            break;
        }
    }
    if( 0 == mCAC.i2AllNum )                                                      // ����Ϊ��ʱ
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
* @brief                 ��Ӵӽڵ�
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
        for( i = 0; i < node_num; i++ )                                                              // ѭ�����DAU
        {
            ASCII_2_HEX_AT( pBuf + addr_index + i * 13, addr_dau, 6 );
            // ���addr
            cac_UserAddDAU( addr_dau );
        }
        cac_CountDAU();                                                                             // ͳ�Ƹ���DAU������Ϣ
        if( FALSE0 == mCAC.bSetup && FALSE0 == mCAC.bOptimize )                                     // ��ǰδ������δά��
        {
            //   drv_Printf("\n��������%d���ڵ㣬�����������ڵ�", pBuf[2]);
            drv_SetTimeDown( 40 );
        }
        uart_Answer_OK();                                                                          // ȷ��Ӧ��
    }
    else
    {
        uart_Answer_ERRORn( ERROR_11H_F1_AT_ADD_NUM_OVER_FLOW );
    }
}

/**
* @brief                  ɾ���ӽڵ�
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
        for( i = 0; i < node_num; i++ )                                                                          //  ѭ��ɾ��DAU
        {
            //flag_del_ok = 0;
            // ����addr
            ASCII_2_HEX_AT( pBuf + addr_index + i * 13, addr_dau, 6 );
            for( j = 0; j < mCAC.i2AllNum; j++ )                                                     //  ������λ��ɾ��DAU��������λ��
            {
                //ִ�� ɾ��addr
                if( mDAU[j].bDUse == TRUE1
                        && mDAU[j].bDWrite == TRUE1
                        && Cb_CmpArry_stat( addr_dau, mDAU[j].aDAddr, 6 ) == TRUE1 )
                {
                    mDAU[j].bDUse = FALSE0;                                                               //  ʹ�ñ�־λ���Ϊ0
                    mDAU[j].bDWrite = FALSE0;                                                            //  ���ر�־λ���Ϊ0
                    mth_ClearRiss( j );                                                                            //  ��ճ�ǿ��
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
        cac_CountDAU();                                                                                     //  ͳ�Ƹ���DAU������Ϣ
        uart_Answer_OK();                                                                                   //  ȷ��Ӧ��
    }
    else
    {
        uart_Answer_ERRORn( ERROR_11H_F2_AT_DEL_NUM_OVER_FLOW );
    }
}

/**
* @brief                  ���������ģ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
                uart_Answer_OK();                                                                                            // ����ȷ��
            }
            return;
        }
    }
    uart_Answer_Invalid_Command();
}


/**
* @brief                  ��������ά������
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN11_Fn101_AT_RTMT( INT8U* pBuf )
{
    // ��û������Ҳû������ά��
    if( FALSE0 == mCAC.bSetup && FALSE0 == mCAC.bOptimize )
    {
        // ����10s��������ά��
        drv_SetTimeDown( 10 );
    }
    // ����ȷ��
    uart_Answer_OK();
}

/**
* @brief                 ��������
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN11_Fn102_AT_RTMS( INT8U* pBuf )
{
    // ���浵��
    cac_SaveAll();
    // ��û������
    if( FALSE0 == mCAC.bSetup )
    {
        // ��Ҫ����
        set_setup_son();
    }
    // ����ȷ��
    uart_Answer_OK();
}


/**
* @brief                 ·����������ʱ��
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
void DL2013_AFN14_Fn2_AT_GCLK( INT8U* pBuf ) // ��ͬ��RTC
{
    INT8U  hour, minute, sec;
    ASCII_2_NUM_AT( pBuf + 8, ( INT32U* )&hour, 2 );
    if( hour > 24 )
    {
        uart_Answer_ERRORn( ERROR_05H_F100_AT_RSSIT_OVER_FLOW );
        return;
    }
    gSysTime.hour = hour;    // ����Сʱ
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
        if( mCAC.bSetup != TRUE1 && nPrioIIFlag.fHourSate != 2 )                                // �������ڵ�
        {
            nPrioIIFlag.fHourSate = 2;
            drv_SetTimeDown( 40 );
        }
    }
    uart_Answer_OK();
}


/**
* @brief                 ��ȡ��ǿ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/08
*/
// ����
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
            // ��������
            uart_Answer_Data( 204 );
            // �ٵȴ�100ms
            drv_Delay10ms( 30 );
            k = 0;
            tPage++;
        }
    }
    // ���һ֡
    if( k > 0 )
    {
        pBuf[2] = tPage;
        pBuf[3] = MAX_DAU / 200 + 1;
        for( ; k < 200; k++ )
        {
            pBuf[4 + k] = 0;
        }
        // ��������
        uart_Answer_Data( 204 );
    }
}


/**
* @brief                 ��ȡ����
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
        // ��ȡ��Ч��
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
            // �ٵȴ�100ms
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
        // �ٵȴ�100ms
        drv_Delay10ms( 30 );
    }
    if( flag_no_data == 0 )
    {
        uart_Answer_ERROR();
    }
}

/**
* @brief                 ��ȡCAC״̬��Ϣ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
    // �����ڵ����
    NUM_2_ASCII_AT( mCAC.i2InNetDAU, pBuf + 3,  &temp_len1 );
    pBuf[temp_len1 + 3] = ',';
    // ���ؽڵ����
    NUM_2_ASCII_AT( mCAC.i2GoodDAU, pBuf + temp_len1 + 4,  &temp_len2 );
    pBuf[temp_len1 + temp_len2 + 4] = ',';
    // ���Ͻڵ����
    NUM_2_ASCII_AT( mCAC.i2BadDAU, pBuf + temp_len1 + temp_len2 + 5,  &temp_len3 );
    pBuf[temp_len1 + temp_len2 + temp_len3 + 5] = '\r';
    // ��������
    drv_UartSend( pBuf, temp_len1 + temp_len2 + temp_len3 + 6 );
}
/**
* @brief                CAC��֤ʹ��
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief               ��ȡ�ڲ��汾��
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief              ��ȡCAC��ʵƵ����
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief           ��ȡ·����Ϣ
                   0:��ȡ0~4��·��
                   1����ȡ5~9��·����Ϣ
                   2: ��ȡ10~12��·����Ϣ
                   3����ȡ��ǰʹ��·����Ϣ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
            pNum = i;    // ��λ�ڵ����
            break;
        }
    }
    // ���� 01 14
    // ��ַ��{������{��ַ����}}*2
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
* @brief           �ƻ���������ʹ��F0H 200
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief              �鿴���ڽ��յĲ����ʵ�ǰ����
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief            ���ô��ڽ��յĲ�����
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief            ���DAU��״̬��Ϣ ���  ��Ե�ͨѶ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/14
*/

void AT_W1( void ) // ���DAU��״̬��Ϣ ���
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
* @brief            ���DAU��״̬��Ϣ ������ ��Ե�ͨѶ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/14
*/
void AT_W2( void ) // ���DAU��״̬��Ϣ ������
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
* @brief            ����͸������  ��Ե� ������
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
* @date       2018/01/14
*/
void AT_W3( void ) // ����͸������  ��Ե� ������
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
* @brief            ����͸������   ���ݵ�Ե㼰·��ģʽ
* @param[in]  pBuf ���պͷ������ݻ�������ָ��
* @author       �����
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
* @brief  ʵ��CFDA-AT�Ľṹ�߼�����
* @param[in] pBuf   ���յ��������ݵ�ָ��
* @param[in] pLen   ���յ��������ݵĳ���
* @return
* @par ʾ��:
* @code
*        DL2013_AT_function��pBuf,100��;
* @endcode
* @deprecated
* @author       �����
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
                    if( memcmp( pBuf + 5, "SIT?\r", 5 ) == 0 ) // ��ѯ��ǿ����
                    {
                        DL2013_AFN03_Fn100_AT_RSSIT( pBuf );
                        return;
                    }
                    else  if( ( memcmp( pBuf + 5, "SIT=", 4 ) == 0 ) && ( ( pBuf[12] == '\r' ) || ( pBuf[11] == '\r' ) ) ) // ���ó�ǿ����
                    {
                        DL2013_AFN05_Fn100_AT_RSSIT( pBuf );
                        return;
                    }
                }
                break;
                case 'I':
                {
                    if( memcmp( pBuf + 5, "V?\r", 3 ) == 0 ) // ��ѯ�ڲ��汾��
                    {
                        DL2013_AFNF0_Fn101_AT_RIV( pBuf );
                        return;
                    }
                }
                break;
                case 'R':
                {
                    if( ( memcmp( pBuf + 5, "PI=", 3 ) == 0 ) && ( pBuf[20] == ',' ) && ( pBuf[22] == '\r' ) ) // ��ѯ�ڲ��汾��
                    {
                        DL2013_AFNF0_Fn106_AT_RRPI( pBuf );
                        return;
                    }
                }
                break;
                case 'A':
                {
                    if( memcmp( pBuf + 5, "R?\r", 3 ) == 0 ) // ��ȡ����
                    {
                        DL2013_AFN0A_Fn52_AT_RAR( pBuf );
                        return;
                    }
                }
                break;
                case 'T':
                {
                    if( memcmp( pBuf + 5, "C?\r", 3 ) == 0 ) // ��ѯ���Ľڵ�ʱ��
                    {
                        DL2013_AFN03_Fn101_AT_RTC( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "C=", 2 ) == 0 ) && ( pBuf[19] == '\r' ) ) // �������Ľڵ�ʱ��
                    {
                        DL2013_AFN05_Fn101_AT_RTC( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "MT\r", 3 ) == 0 ) ) // ����ά��
                    {
                        DL2013_AFN11_Fn101_AT_RTMT( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "RS\r", 3 ) == 0 ) ) // ��������
                    {
                        DL2013_AFN11_Fn102_AT_RTMS( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "ST?\r", 4 ) == 0 ) ) // ��ѯ·������״̬
                    {
                        DL2013_AFN10_Fn4_AT_RTST( pBuf );
                        return;
                    }
                    else if( ( memcmp( pBuf + 5, "MN=", 3 ) == 0 ) ) // ����ά��
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
            if( memcmp( pBuf + 4, "SCL?\r", 5 ) == 0 ) //��ѯ�����ģ
            {
                DL2013_AFN10_Fn100_AT_SSSL( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "SCL=", 4 ) == 0 ) )
            {
                DL2013_AFN11_Fn100_AT_SSSL( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "CCR?\r", 5 ) == 0 ) ) // �鿴״̬�ֺ�ͨ������
            {
                DL2013_AFN03_Fn5_AT_SCCR( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "BRD=12,", 7 ) == 0 ) && ( pBuf[23] == '\r' ) ) // �㲥Уʱ
            {
                DL2013_AFN05_Fn3_AT_SBRD( pBuf );
                return;
            }
        }
        break;
        case 'T':
        {
            if( memcmp( pBuf + 4, "SST=", 4 ) == 0 ) // ���Ͳ�������
            {
                DL2013_AFN04_Fn1_AT_TSST( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "TTH=", 4 ) == 0 ) ) // ����͸�����ƻ�
            {
                DL2013_AFNF0_Fn201_AT_TTTH( pBuf );
                return;
            }
            else if( ( memcmp( pBuf + 4, "TD=", 3 ) == 0 ) ) // ����͸�����ƻ�
            {
                AT_TTD( pBuf );
                return;
            }
        }
        break;
        case 'W':
            if( memcmp( pBuf + 4, "1\r", 2 ) == 0 ) //��� ��Ե����
            {
                AT_W1();
                return;
            }
            else  if( memcmp( pBuf + 4, "2\r", 2 ) == 0 )   //������ ��Ե����
            {
                AT_W2();
                return;
            }
            else  if( memcmp( pBuf + 4, "3\r", 2 ) == 0 ) //������ ��Ե����
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
