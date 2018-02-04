#include    "prio_II.h"
#include        "uart_2013_AT.h"





void        rf_FCtest_Com( void );                  // FC扩展测试
void        rf_Handset_Com( void );
void        rf_YouLiShenQin( void );                // 游离申请
void        rf_YouLiJiuXu( void );                  // 游离就绪
void        rf_Report_Com( void );                  // 主动上报处理


void        rf_TTD_data( void );                  // 游离就绪
void        OpenBoxReport( INT8U* pDAU, INT8U* pBuf );


INT8U       sub_AX5043_set_test_mode( INT8U mode );
void        drv_stm32WDT();
INT8U       CmpParitySum( INT8U* pBuf, INT8U nLength, INT8U nParityChar );
INT8U       Get645FrameType( INT8U* p );

INT8U       uartTxBuf[UART_ARG_1_LEN];                                  // 定义串口发送缓冲区
INT8U       uart_path[36];                                              // 定义路径缓冲区
INT8U       tempBuf22[256];                                             //
INT8U       nHearBuf[20][256];
INT8U       nCallPath[6 * MAX_LAYER];



// 处理串口数据
void tsk_uart()
{
    uart_fun( drvUartRf.aUartRxBuf, drvUartRf.i2UartRxLen );
}

void TSST_AT( INT8U* buf )
{
    INT8U tsst_cmp_buf[5] = { 0x41, 0xcd, 0x0f, 0xff, 0xff };
    if( memcmp( tsst_cmp_buf, buf, 5 ) == 0 )
    {
        if( buf[33] == 0x31  )
        {
            drv_UartSend("\r", 1);
            drv_UartSend( buf + 35, buf[34] );
            drv_UartSend("\r", 1);
        }
    }
}


void RRPI_AT( INT8U* buf , INT16U len)
{
  /*
    INT8U  tsst_cmp_buf[5] = { 0x32, 0, 0x02, 0x4F, 0x4B }, path_channel, temp_len1;
    INT16U i;
    for(i = 0; i < len; i ++)
    {
      if( memcmp( tsst_cmp_buf, buf + i, 5 ) == 0 )
      {

                nMeterUse.fOpen = FALSE0;
       
                
                    mth_WorkYes(tmpDAUNum_RRPI);  
                       // 使用上次成功路径：sNow指向X(0~12)，sNext指向0  //查看上次使用的路径信息。
        path_channel     =    sReadSuccessOldPath(tmpDAUNum_RRPI);  // 读取上次成功路径，成功返回序号，失败返回13
		

    buf[0] = '\r';
    HEX_2_ASCII_AT( dst_addr_RRPI, buf + 1, 6 );
    buf[13] = ',';

    mth_ReadmMath_AT( buf + 14, tmpDAUNum_RRPI, path_channel, (INT8U* )&i, &temp_len1 );
    buf[14 + temp_len1] = '\r';
    drv_UartSend( buf, 15 + temp_len1 );
      }
    }
*/
}
// 处理射频数据
INT8U tsk_rf()
{
    // 协议解析
    if( dl_RfParse( drvUartRf.aRfRxBuf, &mRf ) == TRUE1 )
    {
        // 实时处理的掌机命令
        if( mRf.i1Com == RF_DEV_HAND )
        {
            drvUartRf.i1HandCH = drvUartRf.i1RfCH;
            rf_Handset_Com();
            return FALSE0;
        }
        // 实时处理的调试命令
        if( mRf.i1Com == RF_FC_EXTEND )
        {
            drvUartRf.i1HandCH = drvUartRf.i1RfCH;
            rf_FCtest_Com();
            return FALSE0;
        }
        // 实时处理的主动上报命令
        else if( mRf.i1Com == RF_EVENT_REPT )
        {
            drvUartRf.i1HandCH = drvUartRf.i1RfCH;
            rf_Report_Com();
            return FALSE0;
        }
        // 实时处理开箱报警上报
        else if( mRf.i1Com == RF_OPEN_BOX_REPT )
        {
            OpenBoxReport( mRf.aDAU, mRf.aBuf );
            return FALSE0;
        }
        // 实时处理的游离入网信息
        else if( mRf.i1Com == RF_REQ_INTO_NET )
        {
            rf_YouLiShenQin();
            return FALSE0;
        }
        // 实时处理的游离就绪信息
        else if( mRf.i1Com == RF_REDY_INTO_NET )
        {
            rf_YouLiJiuXu();
            return FALSE0;
        }else if(mRf.i1Com ==  RF_ANS_TTD_DATA)
        {
          rf_TTD_data();
          return FALSE0;
        }
        return TRUE1;
    }
    else
    {
        TSST_AT( drvUartRf.aRfRxBuf );
   //     RRPI_AT( drvUartRf.aRfRxBuf , drvUartRf.i2RfRxLen);
        
    }
    return FALSE0;
}

INT16U FC_Path[MAX_LAYER * 2];
void rf_FCtest_Com()
{
    // 地址不为AA或自己, 不处理
    if( Cb_CmpArry_stat( mRf.aTxID, cAddr0xAA, 6 ) == FALSE0
            && Cb_CmpArry_stat( mRf.aTxID, mCAC.aCAddr, 6 ) == FALSE0 )
    {
        return;
    }
    // mRf.aBuf[0]为命令标识, mRf.aBuf[1]为发送数据长度, 数据从mRf.aBuf[2]开始
    switch( mRf.aBuf[0] )
    {
        // 读取档案
        case 0x24:
        {
            INT8U tN;
            INT16U i;
            INT16U tTop = 0xFFFF;
            INT16U tEnd = 0xFFFF;
            tN = 0;
            for( i = 0; i < mCAC.i2AllNum; i++ )
            {
                if( tTop == 0xFFFF )
                {
                    tTop = i;
                }
                tEnd = i;
                if( dauF_Son( i ) == TRUE1 )
                {
                    mRf.aBuf[6 + 6 * tN] = mDAU[i].aDAddr[0];
                    mRf.aBuf[7 + 6 * tN] = mDAU[i].aDAddr[1];
                    mRf.aBuf[8 + 6 * tN] = mDAU[i].aDAddr[2];
                    mRf.aBuf[9 + 6 * tN] = mDAU[i].aDAddr[3];
                    mRf.aBuf[10 + 6 * tN] = mDAU[i].aDAddr[4];
                    mRf.aBuf[11 + 6 * tN] = mDAU[i].aDAddr[5];
                }
                else
                {
                    mRf.aBuf[6 + 6 * tN] = 0;
                    mRf.aBuf[7 + 6 * tN] = 0;
                    mRf.aBuf[8 + 6 * tN] = 0;
                    mRf.aBuf[9 + 6 * tN] = 0;
                    mRf.aBuf[10 + 6 * tN] = 0;
                    mRf.aBuf[11 + 6 * tN] = 0;
                }
                tN++;
                if( tN >= 30 )
                {
                    mRf.aBuf[2] = ( INT8U )tTop;
                    mRf.aBuf[3] = ( INT8U )( tTop >> 8 );
                    mRf.aBuf[4] = ( INT8U )tEnd;
                    mRf.aBuf[5] = ( INT8U )( tEnd >> 8 );
                    // 返回数据
                    mRf.aBuf[1] = 184;
                    if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                    {
                        drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
                        drv_Delay10ms( 10 );
                    }
                    tN = 0;
                    tTop = 0xFFFF;
                }
            }
            if( tN != 0 )
            {
                mRf.aBuf[2] = ( INT8U )tTop;
                mRf.aBuf[3] = ( INT8U )( tTop >> 8 );
                mRf.aBuf[4] = ( INT8U )tEnd;
                mRf.aBuf[5] = ( INT8U )( tEnd >> 8 );
                // 返回数据
                mRf.aBuf[1] = 4 + tN * 6;
                if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                {
                    drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
                    drv_Delay10ms( 10 );
                }
            }
            mRf.aBuf[1] = 4;
            mRf.aBuf[2] = 0;
            mRf.aBuf[3] = 0;
            mRf.aBuf[4] = 0;
            mRf.aBuf[5] = 0;
            if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
            {
                drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
            }
            break;
        }
        // 读取节点场强
        case 0x25:
        {
            INT16U tDAUN;
            INT16U i;
            INT8U k;
            INT8U tPage;
            INT8U* pRiss;
            tDAUN = mRf.aBuf[2];
            tDAUN = tDAUN << 8;
            tDAUN += mRf.aBuf[1];
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
                    mRf.aBuf[6 + k] = ( *pRiss ) >> 4;
                }
                else
                {
                    mRf.aBuf[6 + k] = ( *pRiss ) & 0x0F;
                    pRiss++;
                }
                k++;
                if( k >= 200 )
                {
                    mRf.aBuf[4] = tPage;
                    mRf.aBuf[5] = MAX_DAU / 200 + 1;
                    mRf.aBuf[2] = ( INT8U )tDAUN;
                    mRf.aBuf[3] = ( INT8U )( tDAUN >> 8 );
                    // 返回数据
                    mRf.aBuf[1] = 204;
                    if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                    {
                        drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
                        drv_Delay10ms( 10 );
                    }
                    k = 0;
                    tPage++;
                }
            }
            // 最后一帧
            if( k > 0 )
            {
                mRf.aBuf[4] = tPage;
                mRf.aBuf[5] = MAX_DAU / 200 + 1;
                mRf.aBuf[2] = ( INT8U )tDAUN;
                mRf.aBuf[3] = ( INT8U )( tDAUN >> 8 );
                for( ; k < 200; k++ )
                {
                    mRf.aBuf[6 + k] = 0;
                }
                // 返回数据
                mRf.aBuf[1] = 204;
                if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                {
                    drv_RfSend( mRf.BufTx, CH_TYPE_TEST_0 );
                }
            }
            break;
        }
        // 读取信道扫描
        case 0x27:
        {
            INT16U tDAUN;
            INT8U pathNo, j;
            tDAUN = mRf.aBuf[2];
            tDAUN = tDAUN << 8;
            tDAUN += mRf.aBuf[1];
            mRf.aBuf[2] = 5;
            for( pathNo = 3; pathNo < 8; pathNo++ )
            {
                mRf.aBuf[3] = pathNo - 3;
                if( mth_3PathWeight( tDAUN, pathNo ) )
                {
                    INT8U tLayer = mth_3PathLayer( tDAUN, pathNo );
                    mRf.aBuf[4] = tLayer + 1;
                    // 拷贝路径
                    Co_SetArry_stat( &mRf.aBuf[5], 0xAA, 6 );
                    mth_3PathGet( FC_Path, tDAUN, pathNo );
                    for( j = 0; j < tLayer - 1; j++ )
                    {
                        Co_CpyArry_stat( mDAU[FC_Path[j]].aDAddr, &mRf.aBuf[11 + 6 * j], 6 );
                    }
                    Co_CpyArry_stat( mDAU[tDAUN].aDAddr, &mRf.aBuf[11 + 6 * j], 6 );
                    // 拷贝场强
                    Co_SetArry_stat( &mRf.aBuf[5 + 6 * ( tLayer + 1 )], 0x00, 8 * ( tLayer + 1 ) );
                    // 遍历路径有效
                    if( pathNo < 7 )
                    {
                        Co_CpyArry_stat( mDAU[tDAUN].aScanRssi[pathNo - 3], &mRf.aBuf[9 + 6 * ( tLayer + 1 )], 8 * tLayer );
                    }
                    else
                    {
                        Co_SetArry_stat( &mRf.aBuf[9 + 6 * ( tLayer + 1 )], 0xFF, 8 * tLayer );
                    }
                    // 返回数据
                    mRf.aBuf[1] = 3 + 14 * ( tLayer + 1 );
                    if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                    {
                        drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
                        drv_Delay10ms( 10 );
                    }
                }
                else
                {
                    // 当前路径节点数
                    mRf.aBuf[4] = 0;
                    // 返回数据
                    mRf.aBuf[1] = 3;
                    if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                    {
                        drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
                        drv_Delay10ms( 10 );
                    }
                }
            }
            // 最短路径
            break;
        }
        // 读取节点底噪
        case 0x28:
        {
            INT16U tDAUN;
            tDAUN = mRf.aBuf[2];
            tDAUN = tDAUN << 8;
            tDAUN += mRf.aBuf[1];
            // 读CAC底噪
            if( tDAUN == 0xFFFF )
            {
                mRf.aBuf[2] = 0xFF;
                mRf.aBuf[3] = 0xFF;
                mRf.aBuf[4] = mCAC.aCBGNoise[0];
                mRf.aBuf[5] = mCAC.aCBGNoise[1];
                mRf.aBuf[6] = mCAC.aCBGNoise[2];
                mRf.aBuf[7] = mCAC.aCBGNoise[3];
                mRf.aBuf[1] = 6;
            }
            else
            {
                mRf.aBuf[2] = ( INT8U )tDAUN;
                mRf.aBuf[3] = ( INT8U )( tDAUN >> 8 );
                Co_CpyArry_stat( mDAU[tDAUN].aDBGNoise, &mRf.aBuf[4], 66 );
                mRf.aBuf[1] = 68;
            }
            // 返回数据
            if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
            {
                drv_RfSend( mRf.BufTx, CH_TYPE_TEST_0 );
            }
            break;
        }
        // 读取信标阶段场强
        case 0x29:
        {
            INT8U tPageNow;
            INT8U tPageAll = ( mCAC.i2Neighbor / 20 ) + 1;
            INT8U i;
            if( ( mCAC.i2Neighbor % 20 ) == 0 )
            {
                tPageAll--;
            }
            mRf.aBuf[2] = tPageAll;
            // 无接收场强
            if( tPageAll == 0 )
            {
                mRf.aBuf[3] = 0;
                mRf.aBuf[4] = 0;
                mRf.aBuf[1] = 3;
                if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                {
                    drv_RfSend( mRf.BufTx, CH_TYPE_TEST_0 );
                }
            }
            for( tPageNow = 0; tPageNow < tPageAll; tPageNow++ )
            {
                INT8U tTop = 20 * tPageNow;
                INT8U tEnd = tTop;
                for( i = 0; i < 20; i++ )
                {
                    if( tEnd < mCAC.i2Neighbor )
                    {
                        Co_CpyArry_stat( mNeighbor[tEnd].aNAddr, &mRf.aBuf[5 + 7 * i], 6 );
                        mRf.aBuf[11 + 7 * i] = mNeighbor[tEnd].aNRssi[0];
                        tEnd++;
                    }
                    else
                    {
                        break;
                    }
                }
                mRf.aBuf[3] = tPageNow;
                mRf.aBuf[4] = tEnd - tTop;
                mRf.aBuf[1] = 3 + 7 * ( tEnd - tTop );
                if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                {
                    drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
                    drv_Delay10ms( 10 );
                }
            }
            break;
        }
        // 读取上下不同路径
        case 0x2A:
        {
            INT16U tDAUN;
            INT8U i, j, pathNo;
            tDAUN = mRf.aBuf[2];
            tDAUN = tDAUN << 8;
            tDAUN += mRf.aBuf[1];
            mRf.aBuf[2] = ( INT8U )tDAUN;
            mRf.aBuf[3] = ( INT8U )( tDAUN >> 8 );
            for( pathNo = 8; pathNo < 10; pathNo += 2 )
            {
                if( mth_3PathWeight( tDAUN, pathNo ) )
                {
                    INT8U tLayer, tLayerUp;
                    // 下行路径
                    mth_3PathGet( FC_Path, tDAUN, pathNo );
                    tLayer = mth_3PathLayer( tDAUN, pathNo );
                    mRf.aBuf[4] = tLayer;
                    for( i = 0; i < tLayer - 1; i++ )
                    {
                        Co_CpyArry_stat( mDAU[FC_Path[i]].aDAddr, &mRf.aBuf[5 + 6 * i], 6 );
                    }
                    // 上行路径
                    mth_3PathGet( FC_Path, tDAUN, pathNo + 1 );
                    tLayerUp = mth_3PathLayer( tDAUN, pathNo + 1 );
                    mRf.aBuf[5 + 6 * i] = tLayerUp;
                    for( j = 0; j < tLayerUp - 1; j++ )
                    {
                        Co_CpyArry_stat( mDAU[FC_Path[j]].aDAddr, &mRf.aBuf[6 + 6 * i], 6 );
                    }
                    mRf.aBuf[1] = 4 + 6 * ( tLayer + tLayerUp - 2 );
                }
                else
                {
                    // 下行路径
                    mRf.aBuf[4] = 0;
                    // 上行路径
                    mRf.aBuf[5] = 0;
                    mRf.aBuf[1] = 4;
                }
                if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                {
                    drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
                    drv_Delay10ms( 10 );
                }
            }
            break;
        }
        // 透传数据
        case 0x41:
        {
            INT8U record = 0, j;
            drvUartRf.i1HandCH = mRf.aBuf[1];
            drv_RfSend( &mRf.aBuf[3], CH_TYPE_HAND );
            drv_SetTimeHear( mRf.aBuf[2] );
            while( drv_WaitTimeHear() == FALSE0 )
            {
                drv_QueryUartRf();
                if( drvUartRf.fRfOK == TRUE1 && record < 20 )
                {
                    INT8U i;
                    nHearBuf[record][0] = ( INT8U )drvUartRf.i2RfRxLen;
                    for( i = 0; i < drvUartRf.i2RfRxLen; i++ )
                    {
                        nHearBuf[record][i + 1] = drvUartRf.aRfRxBuf[i];
                    }
                    record++;
                }
            }
            for( j = 0; j < record; j++ )
            {
                INT8U i;
                mRf.i1Com = 0xFC;
                mRf.aBuf[0] = 0x41;
                mRf.aBuf[1] = nHearBuf[j][0] + 3;
                mRf.aBuf[2] = record;
                mRf.aBuf[3] = j;
                mRf.aBuf[4] = nHearBuf[j][0];
                for( i = 0; i < nHearBuf[j][0]; i++ )
                {
                    mRf.aBuf[i + 5] = nHearBuf[j][i + 1];
                }
                if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                {
                    drv_RfSend( mRf.BufTx, CH_TYPE_TEST_0 );
                    drv_Delay10ms( 30 );
                }
            }
            break;
        }
        default:
            break;
    }
}

// 处理的掌机命令
void rf_Handset_Com()
{
    // 互联互通掌机处理
    if( Cb_CmpArry_stat( mRf.aCAC, cAddr0xAA, 6 ) == TRUE1                             //
            && ( Cb_CmpArry_stat( mRf.aDAU, cAddr0xAA, 6 ) == TRUE1
                 ||  Cb_CmpArry_stat( mRf.aDAU, mCAC.aCAddr, 6 ) == TRUE1 ) )
    {
        INT8U tmpFlag = FALSE0;
        gDtHand* tHand = ( gDtHand* )( mRf.aBuf );
        switch( tHand->i1Com )
        {
            // 设置门限
            case 0x01:
            {
                tmpFlag = TRUE1;
                mCAC.i1Valve = tHand->aBuf[0];
                // 掌机带CAC地址设门限为255时，启动遍历
                if( Cb_CmpArry_stat( mRf.aDAU, mCAC.aCAddr, 6 ) == TRUE1
                        && Cb_CmpArry_stat( mRf.aTxID, mCAC.aCAddr, 6 ) == TRUE1
                        && tHand->aBuf[0] == 0x1C )
                {
                    set_scan();
                }
                tHand->i1Com = 0x02;
                tHand->aBuf[0] = 0;
                tHand->i1Len = 1;
                cac_SaveCAC();
                break;
            }
            // 读取门限
            case 0x03:
            {
                tmpFlag = TRUE1;
                tHand->i1Com = 0x04;
                // 掌机带CAC地址读门限，返回遍历失败数目
                if( Cb_CmpArry_stat( mRf.aDAU, mCAC.aCAddr, 6 ) == TRUE1
                        && Cb_CmpArry_stat( mRf.aTxID, mCAC.aCAddr, 6 ) == TRUE1
                        && mParse.mode_NewScan == TRUE1 )
                {
                    tHand->aBuf[0] = ( INT8U )mCAC.i2ScanErr;
                }
                else
                {
                    tHand->aBuf[0] = mCAC.i1Valve;
                }
                tHand->i1Len = 1;
                break;
            }
            // 下载中继DAU(暂时不做)
            case 0x05:
            {
                break;
            }
            // 删除中继DAU(暂时不做)
            case 0x07:
            {
                break;
            }
            // 空中抄表
            case 0x09:
            {
                INT16U tmpDAUNum;
                // 抄电表全99的时，表示读取CAC的内部版本
                if( Cb_CmpArry_stat( tHand->aBuf, cAddr0x99, 6 ) == TRUE1 )
                {
                    INT8U i;
                    INT8U crcAdd = 0x68;
                    tHand->aBuf[0] = 0;
                    tHand->aBuf[1] = 0x99;
                    tHand->aBuf[2] = 0x99;
                    tHand->aBuf[3] = 0x99;
                    tHand->aBuf[4] = 0x99;
                    tHand->aBuf[5] = 0x99;
                    tHand->aBuf[6] = 0x99;
                    tHand->aBuf[7] = 20;
                    tHand->aBuf[8] = 0x68;
                    tHand->aBuf[9] = 0x99;
                    tHand->aBuf[10] = 0x99;
                    tHand->aBuf[11] = 0x99;
                    tHand->aBuf[12] = 0x99;
                    tHand->aBuf[13] = 0x99;
                    tHand->aBuf[14] = 0x99;
                    tHand->aBuf[15] = 0x68;
                    tHand->aBuf[16] = 0x91;
                    tHand->aBuf[17] = 0x08;
                    tHand->aBuf[18] = 0x33;
                    tHand->aBuf[19] = 0x33;
                    tHand->aBuf[20] = 0x34;
                    tHand->aBuf[21] = 0x33;
                    tHand->aBuf[22] = 0x33 + DT_CUSTOM;
                    tHand->aBuf[23] = 0x33 + DT_EDITIOM_L;
                    tHand->aBuf[24] = 0x33 + DT_EDITIOM_H;
                    tHand->aBuf[25] = 0x33 + DT_PROTOCOL;
                    for( i = 9; i < 26; i++ )
                    {
                        crcAdd += tHand->aBuf[i];
                    }
                    tHand->aBuf[26] = crcAdd;
                    tHand->aBuf[27] = 0x16;
                    tmpFlag = TRUE1;
                    tHand->i1Com = 0x10;
                    tHand->i1Len = 28;
                    break;
                }
                // 正在抄表不执行
                if( mCAC.bMeter == TRUE1 )
                {
                    tmpFlag = TRUE1;
                    tHand->i1Com   = 0x10;
                    tHand->aBuf[0] = 0x01;
                    tHand->i1Len   = 1;
                    break;
                }
                // 查找电表是否在网
                tmpDAUNum = cac_CheckDAUNum( tHand->aBuf );
                if( tmpDAUNum < MAX_DAU
                        && dauF_In( tmpDAUNum ) == TRUE1 )
                {
                    Hand_set_Meter( tmpDAUNum, RF_ASK_METER_DATA, &tHand->aBuf[7], tHand->aBuf[6] );
                    return;
                }
                // 返回不在网
                tmpFlag = TRUE1;
                tHand->i1Com   = 0x10;
                tHand->aBuf[0] = 0x02;
                tHand->i1Len   = 1;
                break;
            }
            // 空中点名
            case 0x11:
            {
                INT16U tmpDAUNum;
                // 正在抄表返回失败
                if( mCAC.bMeter == TRUE1 )
                {
                    tmpFlag = TRUE1;
                    tHand->i1Com   = 0x12;
                    tHand->aBuf[0] = 0x01;
                    tHand->i1Len   = 1;
                    break;
                }
                // 查找电表是否在网
                tmpDAUNum = cac_CheckDAUNum( tHand->aBuf );
                if( ( tmpDAUNum < MAX_DAU )
                        && ( dauF_In( tmpDAUNum ) == TRUE1 ) )
                {
                    Hand_set_Meter( tmpDAUNum, 0xA3, &tHand->aBuf[2], 0 );
                    return ;
                }
                // 返回不在网
                tmpFlag = TRUE1;
                tHand->i1Com   = 0x12;
                tHand->aBuf[0] = 0x02;
                tHand->i1Len   = 1;
                break;
            }
            // 启动组网
            case 0x13:
            {
                if( Cb_CmpArry_stat( mRf.aDAU, mCAC.aCAddr, 6 ) == TRUE1
                        && Cb_CmpArry_stat( mRf.aTxID, mCAC.aCAddr, 6 ) == TRUE1 )
                {
                    tmpFlag = TRUE1;
                    tHand->i1Com = 0x14;
                    if( cacF_Idle() == TRUE1 )
                    {
                        if( tHand->i1Len == 1
                                && tHand->aBuf[0] == 1 )
                        {
                            mCAC.bInitXB = TRUE1;
                        }
                        set_setup_son();
                        tHand->aBuf[0] = 0x01;
                    }
                    else
                    {
                        tHand->aBuf[0] = 0xFF;
                    }
                    tHand->i1Len = 1;
                }
                break;
            }
            // 读取CAC状态
            case 0x15:
            {
                tmpFlag = TRUE1;
                tHand->i1Com = 0x16;
                if( cacF_Idle() == TRUE1 )
                {
                    tHand->aBuf[0] = 1;
                }
                else
                {
                    tHand->aBuf[0] = 0;
                }
                tHand->aBuf[1] = ( INT8U )mCAC.i2DownDAU;
                tHand->aBuf[2] = ( INT8U )( mCAC.i2DownDAU >> 8 );
                tHand->aBuf[3] = ( INT8U )mCAC.i2GoodDAU;
                tHand->aBuf[4] = ( INT8U )( mCAC.i2GoodDAU >> 8 );
                tHand->aBuf[5] = ( INT8U )mCAC.i2BadDAU;
                tHand->aBuf[6] = ( INT8U )( mCAC.i2BadDAU >> 8 );
                tHand->aBuf[7] = DT_EDITIOM_L;
                tHand->aBuf[8] = DT_EDITIOM_H;
                tHand->aBuf[9] = DT_YEAR;
                tHand->aBuf[10] = DT_MOTH;
                tHand->aBuf[11] = DT_DATA;
                tHand->i1Len = 12;
                break;
            }
            // 读取故障节点
            case 0x17:
            {
                INT16U i;
#if         TEST_RETRANSMISSION_COUNT == 1
                INT8U t_N2      =    2;
                tHand->aBuf[3]  = ( INT8U )( mCAC.i4SendCount );
                tHand->aBuf[4]  = ( INT8U )( mCAC.i4SendCount >> 8 );
                tHand->aBuf[5]  = ( INT8U )( mCAC.i4SendCount >> 16 );
                tHand->aBuf[6]  = ( INT8U )( mCAC.i4SucceedCount );
                tHand->aBuf[7]  = ( INT8U )( mCAC.i4SucceedCount >> 8 );
                tHand->aBuf[8]  = ( INT8U )( mCAC.i4SucceedCount >> 16 );
                tHand->aBuf[9]  = ( INT8U )( meter_send );
                tHand->aBuf[10] = ( INT8U )( meter_send >> 8 );
                tHand->aBuf[11] = ( INT8U )( meter_send >> 16 );
                tHand->aBuf[12] = ( INT8U )( meter_succeed );
                tHand->aBuf[13] = ( INT8U )( meter_succeed >> 8 );
                tHand->aBuf[14] = ( INT8U )( meter_succeed >> 16 );
#elif       TEST_RETRANSMISSION_COUNT == 2
                INT8U t_N2      =   1;
                INT16U bcd;
                bcd = Cb_ShortToBCD( mCAC.i2InNetDAU1 );
                tHand->aBuf[3]  = ( INT8U )( bcd );
                tHand->aBuf[4]  = ( INT8U )( bcd >> 8 );
                bcd = Cb_ShortToBCD( mCAC.i2InNetDAU2 );
                tHand->aBuf[5]  = ( INT8U )( bcd );
                tHand->aBuf[6]  = ( INT8U )( bcd >> 8 );
                bcd = Cb_ShortToBCD( mCAC.i2InNetDAU3 );
                tHand->aBuf[7]  = ( INT8U )( bcd );
                tHand->aBuf[8]  = ( INT8U )( bcd >> 8 );
#else
                INT8U t_N2      =    0;
#endif
                for( i = 0; i < mCAC.i2AllNum; i++ )
                {
                    if( dauF_Son( i ) == TRUE1
                            && ( 0 == mDAU[i].b2DNetAll || mDAU[i].bDGood == FALSE0 ) )
                    {
                        tHand->aBuf[3 + t_N2 * 6] = mDAU[i].aDAddr[0];
                        tHand->aBuf[4 + t_N2 * 6] = mDAU[i].aDAddr[1];
                        tHand->aBuf[5 + t_N2 * 6] = mDAU[i].aDAddr[2];
                        tHand->aBuf[6 + t_N2 * 6] = mDAU[i].aDAddr[3];
                        tHand->aBuf[7 + t_N2 * 6] = mDAU[i].aDAddr[4];
                        tHand->aBuf[8 + t_N2 * 6] = mDAU[i].aDAddr[5];
                        t_N2++;
                        if( t_N2 >= 25 )
                        {
                            break;
                        }
                    }
                }
                tHand->i1Com = 0x18;
                tHand->aBuf[0] = ( INT8U )mCAC.i2BadDAU;
                tHand->aBuf[1] = ( INT8U )( mCAC.i2BadDAU >> 8 );
                tHand->aBuf[2] = t_N2;
                tHand->i1Len = 3 + t_N2 * 6;
                tmpFlag = TRUE1;
                break;
            }
            // 设置频道
            case 0x25:
            {
                mCAC.i1BigCH = tHand->aBuf[0];
                mCAC.bSetupCH = tHand->aBuf[1];
                tmpFlag = TRUE1;
                tHand->i1Com = 0x26;
                tHand->aBuf[0] = 0;
                tHand->i1Len = 1;
                cac_SaveCAC();
                break;
            }
            // 读取频道
            case 0x27:
            {
                tmpFlag = TRUE1;
                tHand->i1Com = 0x28;
                tHand->aBuf[0] = mCAC.i1BigCH;
                tHand->aBuf[1] = mCAC.bSetupCH;
                tHand->i1Len = 2;
                break;
            }
            // 直接点名DAU
            case 0x29:
            {
                INT8U tmpCH = tHand->aBuf[0];
                INT8U tmpState, k, rssi;
                INT8U callDAU[6];
                if( tmpCH > 3 )
                {
                    return;
                }
                Co_CpyArry_stat( &tHand->aBuf[1], callDAU, 6 );
                // 路径
                for( k = 0; k < 6 * MAX_LAYER; k++ )
                {
                    nCallPath[k] = 0xFF;
                }
                prio_II_path_set( callDAU, nCallPath, nSetupBuf, 0xA3, 1, CH_TYPE_TEST_0 + tmpCH );
                tmpState = prio_II_path_run();
                // 点名成功
                if( tmpState == PRIO_II_RET_RF )
                {
                    gDtCall* tCall = ( gDtCall* )mRf.aBuf;
                    rssi = tCall->i1RissRx;
                    tHand->i1Com = 0x2A;
                    tHand->aBuf[0] = 1;
                    Co_CpyArry_stat( callDAU, &tHand->aBuf[1], 6 );
                    tHand->aBuf[7] = rssi;
                    tHand->aBuf[8] = nPrioIIEnd.i1Riss;
                    tHand->i1Len = 9;
                    tmpFlag = TRUE1;
                    mRf.i1Com = RF_DEV_HAND;
                    mRf.i1Layer = 1;
                }
                else if( tmpState == PRIO_II_RET_FREE )
                {
                    tHand->i1Com = 0x2A;
                    tHand->aBuf[0] = 0;
                    tHand->i1Len = 1;
                    tmpFlag = TRUE1;
                    mRf.i1Com = RF_DEV_HAND;
                    mRf.i1Layer = 1;
                }
                else
                {
                    tHand->i1Com = 0x2A;
                    tHand->aBuf[0] = 0xFF;
                    tHand->i1Len = 1;
                    tmpFlag = TRUE1;
                    mRf.i1Com = RF_DEV_HAND;
                    mRf.i1Layer = 1;
                }
                break;
            }
            // 启动CAC全网抄表
            case 0x2B:
            {
                if( Cb_CmpArry_stat( mRf.aDAU, mCAC.aCAddr, 6 ) == TRUE1
                        && Cb_CmpArry_stat( mRf.aTxID, mCAC.aCAddr, 6 ) == TRUE1 )
                {
                    tmpFlag = TRUE1;
                    tHand->i1Com = 0x2C;
                    if( cacF_Idle() == TRUE1 )
                    {
                        tHand->aBuf[0] = 0x01;
                        set_meter_all();
                    }
                    else
                    {
                        tHand->aBuf[0] = 0xFF;
                    }
                    tHand->i1Len = 1;
                }
                break;
            }
            // 读取全网抄表结果
            case 0x2D:
            {
                tmpFlag = TRUE1;
                tHand->i1Com = 0x2E;
                // 正在全网抄表
                if( mCAC.bMeterAll )
                {
                    tHand->aBuf[0] = 0x00;
                    tHand->i1Len  = 1;
                }
                else
                {
                    tHand->aBuf[0] = 0x01;
                    tHand->aBuf[1] = ( INT8U )( mCAC.i2MeterAll );
                    tHand->aBuf[2] = ( INT8U )( mCAC.i2MeterAll >> 8 );
                    tHand->aBuf[3] = ( INT8U )( mCAC.i2MeterGood );
                    tHand->aBuf[4] = ( INT8U )( mCAC.i2MeterGood >> 8 );
                    tHand->aBuf[5] = ( INT8U )( mCAC.i2MeterErr );
                    tHand->aBuf[6] = ( INT8U )( mCAC.i2MeterErr >> 8 );
                    tHand->aBuf[7] = ( INT8U )( mCAC.i2Succeed1 );
                    tHand->aBuf[8] = ( INT8U )( mCAC.i2Succeed1 >> 8 );
                    tHand->aBuf[9] = ( INT8U )( mCAC.i2Succeed2 );
                    tHand->aBuf[10] = ( INT8U )( mCAC.i2Succeed2 >> 8 );
                    tHand->aBuf[11] = ( INT8U )( mCAC.i2Succeed3 );
                    tHand->aBuf[12] = ( INT8U )( mCAC.i2Succeed3 >> 8 );
                    tHand->i1Len  = 13;
                }
                break;
            }
            // 读取全网抄表失败节点
            case 0x2F:
            {
                INT16U i;
                INT8U  t_N2 = 0;
                for( i = 0; i < mCAC.i2AllNum; i++ )
                {
                    if( dauF_Son( i ) == TRUE1
                            && ( TRUE1 == mDAU[i].bMeterErr ) )
                    {
                        tHand->aBuf[3 + t_N2 * 6] = mDAU[i].aDAddr[0];
                        tHand->aBuf[4 + t_N2 * 6] = mDAU[i].aDAddr[1];
                        tHand->aBuf[5 + t_N2 * 6] = mDAU[i].aDAddr[2];
                        tHand->aBuf[6 + t_N2 * 6] = mDAU[i].aDAddr[3];
                        tHand->aBuf[7 + t_N2 * 6] = mDAU[i].aDAddr[4];
                        tHand->aBuf[8 + t_N2 * 6] = mDAU[i].aDAddr[5];
                        t_N2++;
                        if( t_N2 >= 25 )
                        {
                            break;
                        }
                    }
                }
                tHand->i1Com = 0x30;
                tHand->aBuf[0] = ( INT8U )mCAC.i2MeterErr;
                tHand->aBuf[1] = ( INT8U )( mCAC.i2MeterErr >> 8 );
                tHand->aBuf[2] = t_N2;
                tHand->i1Len = 3 + t_N2 * 6;
                tmpFlag = TRUE1;
                break;
            }
            // 读取不稳定节点
            case 0x31:
            {
                INT16U i;
                INT8U  t_N2 = 0;
                for( i = 0; i < mCAC.i2AllNum; i++ )
                {
                    if( dauF_Son( i ) == TRUE1
                            && ( TRUE1 == mDAU[i].bInstable ) )
                    {
                        tHand->aBuf[3 + t_N2 * 6] = mDAU[i].aDAddr[0];
                        tHand->aBuf[4 + t_N2 * 6] = mDAU[i].aDAddr[1];
                        tHand->aBuf[5 + t_N2 * 6] = mDAU[i].aDAddr[2];
                        tHand->aBuf[6 + t_N2 * 6] = mDAU[i].aDAddr[3];
                        tHand->aBuf[7 + t_N2 * 6] = mDAU[i].aDAddr[4];
                        tHand->aBuf[8 + t_N2 * 6] = mDAU[i].aDAddr[5];
                        t_N2++;
                        if( t_N2 >= 25 )
                        {
                            break;
                        }
                    }
                }
                tHand->i1Com = 0x32;
                tHand->aBuf[0] = ( INT8U )mCAC.i2Instable;
                tHand->aBuf[1] = ( INT8U )( mCAC.i2Instable >> 8 );
                tHand->aBuf[2] = t_N2;
                tHand->i1Len = 3 + t_N2 * 6;
                tmpFlag = TRUE1;
                break;
            }
            // 透传数据到集中器
            case 0x33:
            {
                if( Cb_CmpArry_stat( mRf.aDAU, mCAC.aCAddr, 6 ) == TRUE1
                        && Cb_CmpArry_stat( mRf.aTxID, mCAC.aCAddr, 6 ) == TRUE1 )
                {
                    uart_ReportData( NULL, tHand->aBuf, tHand->i1Len, 0xF0, 200 );
                }
                break;
            }
            // 配置功能参数
            case 0x35:
            {
                if( Cb_CmpArry_stat( mRf.aDAU, mCAC.aCAddr, 6 ) == TRUE1
                        && Cb_CmpArry_stat( mRf.aTxID, mCAC.aCAddr, 6 ) == TRUE1 )
                {
                    if( tHand->i1Len != 5 )
                    {
                        break;
                    }
                    if( tHand->aBuf[0] != mParse.mode_spread )
                    {
                        // 设置网外蔓延
                        mParse.mode_spread = tHand->aBuf[0];
                        tmpFlag = TRUE1;
                    }
                    if( tHand->aBuf[1] != mParse.mode_NewScan )
                    {
                        // 设置遍历
                        mParse.mode_NewScan = tHand->aBuf[1];
                        tmpFlag = TRUE1;
                    }
                    if( tHand->aBuf[2] != mParse.mode_collect_more )
                    {
                        // 设置增补场强
                        mParse.mode_collect_more = tHand->aBuf[2];
                        tmpFlag = TRUE1;
                    }
                    if( tHand->aBuf[3] != mParse.mode_collect_lost )
                    {
                        // 设置补收场强
                        mParse.mode_collect_lost = tHand->aBuf[3];
                        tmpFlag = TRUE1;
                    }
                    if( tHand->aBuf[4] != mParse.mode_boardcast )
                    {
                        // 设置广播抄表
                        mParse.mode_boardcast = FALSE0;
                        tmpFlag = TRUE1;
                    }
                    if( tmpFlag )
                    {
                        cac_SaveParse();
                    }
                    tmpFlag = TRUE1;
                    tHand->i1Com   = 0x36;
                    tHand->aBuf[0] = 0x01;
                    tHand->i1Len   = 1;
                }
                break;
            }
            // 读取功能参数
            case 0x37:
            {
                if( Cb_CmpArry_stat( mRf.aDAU, mCAC.aCAddr, 6 ) == TRUE1
                        && Cb_CmpArry_stat( mRf.aTxID, mCAC.aCAddr, 6 ) == TRUE1 )
                {
                    tmpFlag = TRUE1;
                    tHand->i1Com = 0x38;
                    tHand->aBuf[0] = mParse.mode_spread;
                    tHand->aBuf[1] = mParse.mode_NewScan;
                    tHand->aBuf[2] = mParse.mode_collect_more;
                    tHand->aBuf[3] = mParse.mode_collect_lost;
                    tHand->aBuf[4] = mParse.mode_boardcast;
                    tHand->i1Len   = 5;
                }
                break;
            }
            // 硬件初始化
            case 0x39:
            {
                switch( tHand->aBuf[0] )
                {
                    // 硬件初始化
                    case 0x00:
                        tHand->i1Com   = 0x3A;
                        tHand->aBuf[0] = 0x01;
                        tHand->i1Len   = 1;
                        if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
                        {
                            drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
                        }
                        cac_SaveAll();
                        drv_Resetcac();
                        break;
                    // 数据区初始化
                    case 0x01:
                        cac_ClearAll();
                        cac_SaveAll();
                        tmpFlag = TRUE1;
                        tHand->i1Com   = 0x3A;
                        tHand->aBuf[0] = 0x01;
                        tHand->i1Len   = 1;
                        break;
                    default:
                        tmpFlag = TRUE1;
                        tHand->i1Com   = 0x3A;
                        tHand->aBuf[0] = 0xFF;
                        tHand->i1Len   = 1;
                        break;
                }
                break;
            }
            // 设置升级标识
            case 0x3B:
            {
                drv_write_updateflag( tHand->aBuf[0] );
                tmpFlag = TRUE1;
                tHand->i1Com   = 0x3C;
                tHand->aBuf[0] = 0x01;
                tHand->i1Len   = 1;
                break;
            }
            // 读取升级标识
            case 0x3D:
            {
                tmpFlag = TRUE1;
                tHand->i1Com   = 0x3E;
                tHand->aBuf[0] = drv_read_updateflag();
                tHand->i1Len   = 0x01;
                break;
            }
        }
        if( tmpFlag == TRUE1 && dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
        {
            drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
        }
    }
}

// 广播校时
void rf_BroadcastTime( INT8U* pBuf, INT8U pLen )
{
    gDtMeter* tM = ( gDtMeter* )( mRf.aBuf );
    INT8U i;
    mRf.i1Com = RF_BROADCAST_TIME;
    mRf.aCAC[0] = mCAC.aCAddr[0];
    mRf.aCAC[1] = mCAC.aCAddr[1];
    mRf.aCAC[2] = mCAC.aCAddr[2];
    mRf.aCAC[3] = mCAC.aCAddr[3];
    mRf.aCAC[4] = mCAC.aCAddr[4];
    mRf.aCAC[5] = mCAC.aCAddr[5];
    tM->i2Len = pLen;
    tM->i1Baud = ( INT8U )mCAC.i2BrdNum;
    for( i = 0; i < pLen; i++ )
    {
        tM->aBuf[i] = pBuf[i];
    }
    // 发送返回命令
    if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
    {
        //drv_RfSend(mRf.BufTx,CH_TYPE_TEST);
        drv_RfSend( mRf.BufTx, CH_TYPE_WORK );
        drv_Printf( "\n========================校时命令已发送===================" );
    }
    else
    {
        drv_Printf( "\n================ =====校时命令发送失败===================" );
    }
}

// 广播透传
void rf_BroadcastData( INT8U* pBuf, INT8U pLen )
{
    gDtMeter* tM = ( gDtMeter* )( mRf.aBuf );
    INT8U i;
    mRf.i1Com = RF_BROADCAST_DATA;
    mRf.aCAC[0] = mCAC.aCAddr[0];
    mRf.aCAC[1] = mCAC.aCAddr[1];
    mRf.aCAC[2] = mCAC.aCAddr[2];
    mRf.aCAC[3] = mCAC.aCAddr[3];
    mRf.aCAC[4] = mCAC.aCAddr[4];
    mRf.aCAC[5] = mCAC.aCAddr[5];
    tM->i2Len = pLen;
    tM->i1Baud = ( INT8U )mCAC.i2BrdNum;
    for( i = 0; i < pLen; i++ )
    {
        tM->aBuf[i] = pBuf[i];
    }
    // 发送返回命令
    if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
    {
        //drv_RfSend(mRf.BufTx,CH_TYPE_TEST);
        drv_RfSend( mRf.BufTx, CH_TYPE_WORK );
        drv_Printf( "\n========================校时命令已发送===================" );
    }
    else
    {
        drv_Printf( "\n================ =====校时命令发送失败===================" );
    }
}

// 游离申请响应
void rf_YouLiShenQin()
{
    INT16U tmpN = cac_CheckDAUNum( mRf.aDAU );
    // 队列有该节点记录且节点是下载节点
    if( tmpN < MAX_DAU && TRUE1 == dauF_Son( tmpN ) )
    {
        gDtAnswerNew* tYL   = ( gDtAnswerNew* )( mRf.aBuf );
        // 主节点地址
        mRf.aCAC[0]         =   mCAC.aCAddr[0];
        mRf.aCAC[0]         =   mCAC.aCAddr[0];
        mRf.aCAC[0]         =   mCAC.aCAddr[0];
        mRf.aCAC[0]         =   mCAC.aCAddr[0];
        mRf.aCAC[0]         =   mCAC.aCAddr[0];
        mRf.aCAC[0]         =   mCAC.aCAddr[0];
        // 上行场强
        tYL->iYouLiUpRiss   =   drvUartRf.i1RfRiss;
        // 命令字：游离入网请求响应
        mRf.i1Com           =   RF_REP_INTO_NET;
        if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
        {
            drv_RfSend( mRf.BufTx, CH_TYPE_WORK );
        }
    }
}
void rf_TTD_data()
{
  
            drv_UartSend(mRf.aBuf + 1 , mRf.aBuf[0]  );
      

}
// 游离就绪
void rf_YouLiJiuXu()
{
    INT8U  m;                                              // 1B索引，如层次
    INT16U i;                                              // 2B索引，如序号
    INT16U tmPath[MAX_LAYER];                              // 缓存游离上报路径
    INT16U tmpN = 0;                                       // 序号
    INT8U  tF  =   FALSE0;                                 // 需要配置标识
    gDtReady* tYL = ( gDtReady* )( mRf.aBuf );
    // 层次异常
    if( tYL->i1Layer > MAX_LAYER )
    {
        return;
    }
    // 游离上报路径中继节点缓存
    for( m = 0; m < tYL->i1Layer - 1; m++ )
    {
        // 工作频道上报，中继节点必是本网，即tmpN有效
        tmpN = cac_CheckDAUNum( tYL->aPath + 6 * m );
        tmPath[m] = tmpN;
    }
    // 查找游离节点位置
    tmpN = cac_CheckDAUNum( tYL->aDAU );
    tmPath[tYL->i1Layer - 1] = tmpN;
    // 缓存层次，供3Path使用
    mDAU[tmpN].b4DLayerSon = tYL->i1Layer;
    // 若中继索引是1~6，表示是第2-7层节点发出
    if( mRf.i1NoteID < MAX_LAYER )
    {
        // 设置发送时间
        drv_SetTimeDAU( mRf.i1NoteID );
        // 等待超时
        while( drv_WaitTimeDAU() == FALSE0 )
        {
        }
    }
    // 位置有效，收场强处理
    if( TRUE1 == dauF_Down( tmpN ) )
    {
        gDtRiss* tRiss = ( gDtRiss* )( nSetupBuf );
        // 只收第0页
        tRiss->i1PageNow = 0;
        // 需要上行场强
        tRiss->fUpRiss   = TRUE1;
        // 指定路径测试(使用游离上报路径)
        prio_II_path_set( tYL->aDAU, tYL->aPath, nSetupBuf, RF_COLLECT_RISS, 2, CH_TYPE_TEST );
        // 收集场强成功
        if( PRIO_II_RET_RF == prio_II_path_run() )
        {
            INT16U tDAUN = 0;
            INT8U  tRValUp = 0;
            INT8U  tRValDown = 0;
            // 填写上报路由下行场强
            for( i = 0; i < tRiss->i1DAUNum; i++ )
            {
                // 下行场强值
                tRValDown = cac_DAURissToVal( tRiss->aDownRiss[7 * i + 6] );
                // 获取场强表节点序号
                tDAUN = cac_CheckDAUNum( &( tRiss->aDownRiss[7 * i] ) );
                // 保存到场强表
                if( tDAUN == 0xFFFF || tDAUN < MAX_DAU )
                {
                    // 写下行场强
                    mth_WriteRiss( tmpN, tDAUN, tRValDown );
                    // 若有上行场强表
                    if( TRUE1 == tRiss->fUpRiss )
                    {
                        tRValUp   = cac_DAURissToVal( tRiss->aUpRiss[i] );
                        mth_WriteRiss( tDAUN, tmpN, tRValUp );
                    }
                }
            }
            // 需要配置标识置位
            tF = TRUE1;
        }
        // 场强收集路径处理
        mth_YouLiRissOK( tmpN, tmPath );
        // 计算三条路径
        mth_3PathInit( tmpN );
        for( i = 0; i < mCAC.i2AllNum; i++ )
        {
            if( TRUE1 == dauF_Down( i ) )
            {
                mth_3PathSet( i );
            }
        }
        for( m = 1; m < MAX_LAYER; m++ )
        {
            // 若有路径返回，退出循环
            if( mth_3PathRunEnd( m, FALSE0 ) )
            {
                mDAU[tmpN].b4DLayerSon  = m;
                break;
            }
        }
        // 游离节点在档案内
        if( TRUE1 == dauF_Son( tmpN )
                && TRUE1 == tF )
        {
            gDtSetDAU* tSet = ( gDtSetDAU* )( nSetupBuf );
            // 工作频道
            tSet->i1CH      =   mCAC.i1BigCH;
            // 层次
            tSet->i1Layer   =   mDAU[tmpN].b4DLayerSon;
            // 时隙
            tSet->i2ShiXi   =   mCAC.i2ShiXi++;
            // 不清场强表
            tSet->i1Type    =   1;
            // 短地址
            cac_GetShortID( mDAU[tmpN].aDShortID, mDAU[tmpN].aDAddr );
            tSet->aShortID[0]   =   mDAU[tmpN].aDShortID[0];
            tSet->aShortID[1]   =   mDAU[tmpN].aDShortID[1];
            // 使用工作路径测试：配置DAU
            prio_II_work_set( mDAU[tmpN].aDAddr, nSetupBuf, RF_ASK_CONFIGURE_DAU, 2, tmpN, CH_TYPE_TEST );
            // 配置成功
            if( TRUE1 == prio_II_work_run() )
            {
                // 游离节点收集回场强就标记成功
                mDAU[tmpN].bDGood       =   TRUE1;
                mDAU[tmpN].b2DNetAll    =   NET_ALL_TIME;
#ifdef      ONLY_RF_STATE_GRID2013
                mDAU[i].aDVersion[0]    =   tSet->Version[0];
                mDAU[i].aDVersion[1]    =   tSet->Version[1];
#endif
                // 更新节点个数信息
                cac_CountDAU();
                // 保存所有更新
                cac_SaveAll();
            }
        }
    }
}

// 主动上报：串口部分处理
void rf_Report_Com()
{
    INT16U i, j, tPath[MAX_LAYER];
    i = cac_CheckDAUNum( mRf.aDAU );
    if( i == MAX_DAU )
    {
        return;
    }
    mth_WorkInit( i, mDAU[i].b4DLayerSon );
    for( j = 0; j < mCAC.i2AllNum; j++ )
    {
        if( dauF_In( j ) == TRUE1 )
        {
            mth_WorkSet( j );
        }
    }
    if( mth_WorkRun( tPath, FALSE0, ( INT8U* )0 ) == FALSE0 )
    {
        return;
    }
    cac_2ByteTo6Byte( tPath, uart_path, FALSE0 );
    mRf.aPath = uart_path;
    mRf.i1Layer = cac_LayerInPath( mRf.aDAU, mRf.aPath );
    mRf.aCAC[0] = mCAC.aCAddr[0];
    mRf.aCAC[1] = mCAC.aCAddr[1];
    mRf.aCAC[2] = mCAC.aCAddr[2];
    mRf.aCAC[3] = mCAC.aCAddr[3];
    mRf.aCAC[4] = mCAC.aCAddr[4];
    mRf.aCAC[5] = mCAC.aCAddr[5];
    if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
    {
        drv_RfSend( mRf.BufTx, CH_TYPE_HAND );
        if( mCAC.bReport == TRUE1 )
        {
#if PRO_UART == 2007
            uart_ReportData( mRf.aDAU, &mRf.aBuf[1], mRf.aBuf[1] + 1, 6, 2 );
#elif PRO_UART == 2013
            uart_ReportData( mRf.aDAU, &mRf.aBuf[1], mRf.aBuf[1] + 1, 6, 5 );
#endif
        }
    }
}

// ============================================================================
// 北京测试命令：68 AA AA AA AA AA AA 68 04 08 4A 42 00 4B 45 50 01 10 55 16
//                                                                                                                                          t
// ============================================================================
void rf_BJ_Test( INT8U* xbuf )
{
    if( xbuf[0] == 0x68
            && xbuf[1] == 0xAA
            && xbuf[2] == 0xAA
            && xbuf[3] == 0xAA
            && xbuf[4] == 0xAA
            && xbuf[5] == 0xAA
            && xbuf[6] == 0xAA
            && xbuf[7] == 0x68
            && xbuf[8] == 0x04
            && xbuf[9] == 0x08
            && xbuf[10] == 0x4a
            && xbuf[11] == 0x42
            && xbuf[12] == 0x00
            && xbuf[13] == 0x4B
            && xbuf[14] == 0x45
            && xbuf[15] == 0x50
            && xbuf[19] == 0x16 )
    {
        INT8U i;
        INT8U crc = 0;
        for( i = 0; i < 18; i++ )
        {
            crc += xbuf[i];
        }
        if( crc == xbuf[18] )
        {
            sub_AX5043_set_test_mode( 0x80 + mCAC.i1BigCH );
            for( i = 0; i < xbuf[17]; i++ )
            {
                drv_stm32WDT();
                drv_Delay10ms( 100 );
            }
            sub_AX5043_set_test_mode( 0xff );
        }
    }
}

// ============================================================================
// 国网1374.4测试命令：68 AA AA AA AA AA AA 68 C  L  Time CH CS 16
//        示例       ：68 AA AA AA AA AA AA 68 04 02  10  01 E3 16                                                                                                                                  t
// ============================================================================
void rf_1374_3_Test( INT8U* xbuf )
{
    if( xbuf[0] == 0x68
            && xbuf[1] == 0xAA
            && xbuf[2] == 0xAA
            && xbuf[3] == 0xAA
            && xbuf[4] == 0xAA
            && xbuf[5] == 0xAA
            && xbuf[6] == 0xAA
            && xbuf[7] == 0x68 )
    {
        INT8U i = 0;
        //  操作码
        switch( xbuf[8] )
        {
            //  发射测试
            case 0x04:
                if( CmpParitySum( xbuf, xbuf[9] + 10, xbuf[12] ) == TRUE1 )
                {
                    uart_Answer_Yes();
                    //  频率超过最大索引
                    if( xbuf[11] > 66 )
                    {
                        xbuf[11] %= 66;
                    }
                    //  发射频道
                    sub_AX5043_set_test_mode( 0x80 + xbuf[11] );
                    //  发射时间
                    for( i = 0; i < xbuf[10]; i++ )
                    {
                        drv_Delay10ms( 100 );
                        drv_stm32WDT();
                    }
                    //  发射模式
                    sub_AX5043_set_test_mode( 0xff );
                }
                break;
            //  接收测试
            case 0x05:
                if( CmpParitySum( xbuf, xbuf[9] + 10, xbuf[xbuf[9] + 10] ) == TRUE1 )
                {
                    uart_Answer_Yes();
                    //  频率超过最大索引
                    if( xbuf[10] > 66 )
                    {
                        xbuf[10] %= 66;
                    }
                    //  接收模式
                    sub_AX5043_set_test_mode( xbuf[10] & 0x7f );
                    //  接收时间
                    for( i = 0; i < 240; i++ )
                    {
                        drv_Delay10ms( 100 );
                        drv_stm32WDT();
                    }
                    //  接收模式
                    sub_AX5043_set_test_mode( 0xff );
                }
                break;
            default:
                break;
        }
    }
}

// ============================================================================
// 国网射频发送测试命令：68 12 00 4A 00 00 00 00 00 00 04 08 00  Ch Type Time CS 16
//        示例         ：68 12 00 4A 00 00 00 00 00 00 04 08 00  01  04   10  6B 16                                                                                                                                  t
// ============================================================================
void rf_SendTest( INT8U* xbuf )
{
    if( xbuf[0] == 0x68
            && xbuf[1] == 0x12
            && xbuf[2] == 0x00
            && xbuf[3] == 0x4a
            && xbuf[4] == 0x00
            && xbuf[5] == 0x00
            && xbuf[6] == 0x00
            && xbuf[7] == 0x00
            && xbuf[8] == 0x00
            && xbuf[9] == 0x00
            && xbuf[10] == 0x04
            && xbuf[11] == 0x08
            && xbuf[12] == 0x00
            && xbuf[17] == 0x16 )
    {
        INT8U i;
        INT8U crc = 0;
        for( i = 3; i < 16; i++ )
        {
            crc += xbuf[i];
        }
        if( crc == xbuf[16] )
        {
            uart_Answer_Yes();
            //  频率超过最大索引
            if( xbuf[13] > 66 )
            {
                xbuf[13] %= 66;
            }
            //  发射频道
            //sub_adf7020_set_test_mode2(0x80 + xbuf[13]);
            sub_AX5043_set_test_mode( 0x80 + xbuf[13] );
            for( i = 0; i < xbuf[15]; i++ )
            {
                drv_stm32WDT();
                drv_Delay10ms( 100 );
            }
            //  退出发射模式
            //sub_adf7020_set_test_mode2(0xff);
            sub_AX5043_set_test_mode( 0xff );
        }
    }
}

// 计算同步累加和，与校验字符比较，相同返回TRUE1
INT8U CmpParitySum( INT8U* pBuf, INT8U nLength, INT8U nParityChar )
{
    INT8U i = 0;
    INT8U lucParitySum = 0;
    //  计算校验和
    for( i = 0; i < nLength; i++ )
    {
        lucParitySum += pBuf[i];
    }
    //  等于校验字
    if( lucParitySum == nParityChar )
    {
        return TRUE1;
    }
    return FALSE0;
}

//==========================================================================
// 函数名称：Get645FrameType
// 函数功能：获取645帧类型
// 入口参数：数据帧起始地址
// 出口参数：0 自适应   1 97规约  2  07规约 0xff  未知规约
//==========================================================================
INT8U Get645FrameType( INT8U* p )
{
    INT8U flag = 2;                                                                       //  默认07表
    INT8U lenth = 0;
    for( lenth = 0; lenth < 6; lenth++ )                                       //  过滤FE
    {
        if( p[lenth] == 0x68 )
        {
            break;
        }
    }
    if( ( p[lenth + 0] == 0x68 )                                                    //  判断帧头
            && ( p[lenth + 7] == 0x68 )
            && ( !( p[lenth + 8] & 0x40 ) ) )                                          //  命令字错误标志为0
    {
        if( ( p[lenth + 8] & 0x1f ) == 0x01 )
        {
            flag = 1;                                                           //97规约
        }
        else if( ( p[lenth + 8] & 0x1f ) == 0x02 )
        {
            flag = 1;                                                           //97规约
        }
        else if( ( p[lenth + 8] & 0x1f ) == 0x03 )
        {
            if( p[lenth + 9] < 5 )                                              //数据长度小于5为97，否则为07
            {
                flag = 1;
            }
        }
        else if( ( p[lenth + 8] & 0x1f ) == 0x04 )
        {
            flag = 1;                                                           //97规约
        }
        else if( ( p[lenth + 8] & 0x1f ) == 0x0a )
        {
            flag = 1;                                                           //97规约
        }
        else if( ( p[lenth + 8] & 0x1f ) == 0x0c )
        {
            flag = 1;                                                           //97规约
        }
        else if( ( p[lenth + 8] & 0x1f ) == 0x0f )
        {
            flag = 1;                                                           //97规约
        }
        else if( ( p[lenth + 8] & 0x1f ) == 0x10 )
        {
            flag = 1;                                                           //97规约
        }
        else if( ( p[lenth + 8] & 0x1f ) == 0x08 )
        {
            flag = 0;                                                           //07 97规约都有可能
        }
    }
    else
    {
        flag = 0;
    }
    return flag;                                                                //
}

//===============================================================================================
//  函数名称:        uart_DAUType
//  函数描述:           DAU节点类型判断
//  入口参数:
//
//             1.  pNum    <类型>  INT16U                      取值范围0 ~ n，n为DAU序列下标最大值
//                                 <说明>  DAU节点在序列中的编号
//             2.  pType    <类型>  INT8U
//                                 <说明>  节点类型  0     在网
//                                                                              1     不在网
//                                                                              2     故障
//                                                                              3     下载
//                                                                              5     发现
//                                                                              6      微功率无线
//
//  返回值  :           TRUE1 表示是，FALSE0表示不是
//  说明    ：
//              1.    数据单元格式：pBuf[0]~pBuf[1]  从节点起始序号
//                                                            pBuf[2]           从节点数量
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>
//                               陈炎    2013-09-30           1.0           添加注释
//===============================================================================================
INT8U uart_DAUType( INT16U pNum, INT8U pType )
{
    switch( pType )
    {
        // 在网电表信息
        case 0:
            if( dauF_Good( pNum ) == TRUE1 && mDAU[pNum].bDFind == FALSE0 )
            {
                return TRUE1;
            }
            return FALSE0;
        // 不在网、故障电表信息
        case 1:
        case 2:
            if( dauF_Bad( pNum ) == TRUE1 && mDAU[pNum].bDFind == FALSE0 )
            {
                return TRUE1;
            }
            return FALSE0;
        // 下载电表信息
        case 3:
            if( dauF_Son( pNum ) == TRUE1 )
            {
                return TRUE1;
            }
            return FALSE0;
        // 发现节点
        case 5:
            //      if(dauF_else(pNum)==TRUE1)
            //          return TRUE1;
            return FALSE0;
        //  微功率无线
        case 6:
            if( dauF_Son( pNum ) == TRUE1 )
            {
                return TRUE1;
            }
            return FALSE0;
        default:
            return FALSE0;
    }
}

// 路径处理函数
void uart_Path2ByteTo6Byte( INT16U pNum, INT8U* pPath )
{
    INT16U* p2ByteNum;
    INT8U i;
    INT8U j;
    for( i = 0; i < 6; i++ )
    {
        p2ByteNum = ( INT16U* )( pPath + i * 36 );
        for( j = 0; j < 6; j++ )
        {
            if( ( *p2ByteNum ) >= MAX_DAU
                    || ( *p2ByteNum ) == pNum
                    || dauF_In( ( *p2ByteNum ) ) == FALSE0
              )
            {
                break;
            }
            uart_path[j * 6 + 0] = mDAU[*p2ByteNum].aDAddr[0];
            uart_path[j * 6 + 1] = mDAU[*p2ByteNum].aDAddr[1];
            uart_path[j * 6 + 2] = mDAU[*p2ByteNum].aDAddr[2];
            uart_path[j * 6 + 3] = mDAU[*p2ByteNum].aDAddr[3];
            uart_path[j * 6 + 4] = mDAU[*p2ByteNum].aDAddr[4];
            uart_path[j * 6 + 5] = mDAU[*p2ByteNum].aDAddr[5];
            p2ByteNum++;
        }
        for( ; j < 6; j++ )
        {
            uart_path[j * 6 + 0] = 0;
            uart_path[j * 6 + 1] = 0;
            uart_path[j * 6 + 2] = 0;
            uart_path[j * 6 + 3] = 0;
            uart_path[j * 6 + 4] = 0;
            uart_path[j * 6 + 5] = 0;
        }
        for( j = 0; j < 36; j++ )
        {
            pPath[i * 36 + j] = uart_path[j];
        }
    }
}

// 开箱报警上报
void OpenBoxReport( INT8U* pDAU, INT8U* pBuf )
{
    INT16U  i2DAUSeq = MAX_DAU;
    INT16U  i;
    // 计算节点序号
    i2DAUSeq = cac_CheckDAUNum( pDAU );
    // 若不在档案
    if( i2DAUSeq >= MAX_DAU )
    {
        drv_Printf( "\n上报的DAU为 " );
        drv_PrintfDAU( pDAU );
        drv_Printf( "不在档案中" );
    }
    else
    {
        mRf.aCAC[0] = mCAC.aCAddr[0];
        mRf.aCAC[1] = mCAC.aCAddr[1];
        mRf.aCAC[2] = mCAC.aCAddr[2];
        mRf.aCAC[3] = mCAC.aCAddr[3];
        mRf.aCAC[4] = mCAC.aCAddr[4];
        mRf.aCAC[5] = mCAC.aCAddr[5];
        mRf.aDAU[0] = pDAU[0];
        mRf.aDAU[1] = pDAU[1];
        mRf.aDAU[2] = pDAU[2];
        mRf.aDAU[3] = pDAU[3];
        mRf.aDAU[4] = pDAU[4];
        mRf.aDAU[5] = pDAU[5];
        // 射频处理
        if( dl_RfStruct( mRf.BufTx, &mRf ) == TRUE1 )
        {
            drv_Printf( "\n接收到开箱事件上报，事件序号%d，射频应答构造成功", pBuf[0] );
            drv_RfSend( mRf.BufTx, CH_TYPE_WORK );
        }
        else
        {
            drv_Printf( "\n接收到开箱事件上报，事件序号%d，射频应答构造失败", pBuf[0] );
        }
        // 串口处理
        drv_Printf( "\nDAU = " );
        drv_PrintfDAU( pDAU );
        drv_Printf( "上报开箱事件" );
        tempBuf22[0] = mDAU[i2DAUSeq].aDProType[1]; // 协议
        tempBuf22[1] = pDAU[0];                     // DAU地址
        tempBuf22[2] = pDAU[1];
        tempBuf22[3] = pDAU[2];
        tempBuf22[4] = pDAU[3];
        tempBuf22[5] = pDAU[4];
        tempBuf22[6] = pDAU[5];
        tempBuf22[7] = pBuf[1];                     // 报文长度
        for( i = 0; i < pBuf[1]; i++ )
        {
            tempBuf22[i + 8] = pBuf[i + 2];
        }
        // 串口06H F20上报
        uart_ReportData( pDAU, tempBuf22, pBuf[1] + 8, 6, 20 );
    }
}

