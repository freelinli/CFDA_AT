#include	"prio_II.h"
#include        "uart_2013_AT.h"

#if PRO_UART == 2013
extern INT8U gi1UpSeqNum;                                                                       // 延时响应类报文序号
extern INT8U gi1DownSeqNum;                                                                     // 下行报文序号
extern INT8U gi1ReportSeqNum;                                                                   // 上报报文序号
#endif

#define         MIN_NOISE          7                                                            // 抄表低噪门限
static INT8U    sState;                                                                         // 抄表频道选择
INT32U          meter_send;
INT32U          meter_succeed;
INT8U		    reportDAU[7];                                                                   // 故障上报节点记录
INT8U			reportPath[6*MAX_LAYER];                                                        // 上报路径
INT8U			FlgReceiveData;                                                                 // 抄表应答标志 

INT8U           meter_ChooseCH              (INT8U xTime);                                      // 抄表频道选择 
INT8U           meter_ChooseCH3             (INT16U *pPath, INT8U pLayer);
INT8U			meter_one_check				(INT8U pF, INT8U again);						                    // 单次抄表
void			WaitTransmitTime		    (void);							                    // 等待传输时延
INT8U			meter_waitTime				(INT8U pLayer);		                // 抄表时间等待
void            meter_broadcast             (void);
void            meter_broadcast_again       (void);
INT8U           meter_board_divisor         (INT16U pCapacity, INT8U pTime);

INT8U           meterBuf[LEN_USER];

extern INT8U   flag_at_meter;


gMeterUse nMeterUse;						// 抄表参数结构 


INT8U tmpDAUNum_RRPI = 0;
INT8U dst_addr_RRPI[6];

// 抄表初始化
void init_Meter()
{
    //  重置抄表模式
    INT16U i;
    for (i=0; i<MAX_DAU; i++)
    {
        mDAU[i].b2DMeter = 0;
        mDAU[i].bDMode = METER_BOARD;
        mDAU[i].bDUpDown = FALSE0;
        if(dauF_In(i) == TRUE1)
        {
            mDAU[i].bDMode=METER_PATH;
#if   	MODE_UP_DOWN
            mDAU[i].bDUpDown=TRUE1;
#endif
        }
    }

    //  清空抄表结构体
    Co_SetArry_stat((INT8U*)(&nMeterUse),0,sizeof(gMeterUse));
    nMeterUse.pBuf  = drvBuf.pmMeter;
}

// 掌机启动抄表流程
void Hand_set_Meter(INT16U pNum,INT8U pCom,INT8U *pBuf,INT16U pLen)
{
    INT16U i;
    gDtMeter *tMeter=(gDtMeter *)(nMeterUse.pBuf);

    // 拷贝数据
    nMeterUse.aDAU[0]=mDAU[pNum].aDAddr[0];
    nMeterUse.aDAU[1]=mDAU[pNum].aDAddr[1];
    nMeterUse.aDAU[2]=mDAU[pNum].aDAddr[2];
    nMeterUse.aDAU[3]=mDAU[pNum].aDAddr[3];
    nMeterUse.aDAU[4]=mDAU[pNum].aDAddr[4];
    nMeterUse.aDAU[5]=mDAU[pNum].aDAddr[5];

    tMeter->i1Baud=0;
    tMeter->i2Len=pLen;

    for(i=0;i<pLen;i++)
        tMeter->aBuf[i]=pBuf[i];

    nMeterUse.i2Len=pLen;
    nMeterUse.i1XieYi=0;
    nMeterUse.i1Bo=0;
    nMeterUse.i2Num=pNum;
    nMeterUse.i1Event=pCom;
    nMeterUse.fHand = TRUE1;
    nMeterUse.fUseYL = FALSE0;

    drv_SetTimeAll(mCAC.i1ReadMeterMaxTime - 10);
    nMeterUse.fOpen=TRUE1;
}

//===============================================================================================
//  函数名称:        set_Meter
//  函数描述:   	    启动抄表流程
//  入口参数:	 
//
//             1.  pAddr    <类型>  INT8U*
//                                 <说明>  字符串指针，指向从节点地址
//             2.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向645帧
//             3.  pLen       <类型>  INT16U
//                                 <说明>  字符串长度
//             4.  pXie       <类型>  INT8U
//                                 <说明>  协议类型，暂未使用 
//             5.  pBo        <类型>  INT8U
//                                 <说明>  波特率
//          
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                 陈 炎    2013-09-25           1.0           生成函数
//                           陈 炎    2013-12-10           1.0           修改注释
//===============================================================================================
void set_Meter(INT16U pNum, INT8U *pBuf, INT16U pLen, INT8U pXie, INT8U pBo, INT8U pCom)
{
    INT16U i;
  
    nMeterUse.aDAU[0]=mDAU[pNum].aDAddr[0];                                          //  保存信息到抄表参数结构
    nMeterUse.aDAU[1]=mDAU[pNum].aDAddr[1];
    nMeterUse.aDAU[2]=mDAU[pNum].aDAddr[2];
    nMeterUse.aDAU[3]=mDAU[pNum].aDAddr[3];
    nMeterUse.aDAU[4]=mDAU[pNum].aDAddr[4];
    nMeterUse.aDAU[5]=mDAU[pNum].aDAddr[5];
    nMeterUse.i2Len=pLen;
    nMeterUse.i1XieYi=pXie;
    nMeterUse.i1Bo=pBo;  
    nMeterUse.i2Num   = pNum;
    nMeterUse.fHand   = FALSE0;
    nMeterUse.fUseYL  = FALSE0;
    nMeterUse.fBoard  = FALSE0;
    nMeterUse.i1Event = pCom;
    
    // 抄表模式为广播, 广播失败次数少于3次
    if (mDAU[pNum].bDMode==METER_BOARD && mDAU[pNum].b2DMeter<3)
        nMeterUse.fBoard = TRUE1;
    // 抄表模式为广播, 广播失败次数为3次
    else if (mDAU[pNum].bDMode==METER_BOARD && mDAU[pNum].b2DMeter==3)
    {
#ifdef  _LD_RELEASE
        if(dauF_In(pNum)==TRUE1)
        {
            nMeterUse.fBoard = TRUE1;
        }
        else
        {
            uart_Answer_Unknown();
            return;
        }
#else
        nMeterUse.fUseYL=TRUE1;
        Co_SetArry_stat(nMeterUse.aPath,0,36);
#endif
    }

#if PRO_UART == 2013
    gi1UpSeqNum = gi1DownSeqNum;                                                              //  确认会抄表才修改上行报文序号
#endif
    
    // 备份抄表数据
    for(i=0; i<pLen; i++)
        meterBuf[i] = pBuf[i];
    
    // 广播抄表
    if (nMeterUse.fBoard)
    {
        if(mParse.mode_boardcast)
        {
            gDtBroadMeter *tMeter = (gDtBroadMeter*)nMeterUse.pBuf;
            INT16U  tCapacity = mCAC.i2DownDAU;
            
            if(tCapacity<30)
                tCapacity = 30;
            
            if ((mCAC.i2BrdNum&0x00FF)<2)
                mCAC.i2BrdNum |= 0x0002;
            tMeter->i1BroadNO = (INT8U)mCAC.i2BrdNum++;
            tMeter->i2Shixi   = 0;
            tMeter->i1Divisor = 1 + mDAU[pNum].b3Divisor;   //meter_board_divisor(tCapacity, 55);
            tMeter->i2Capacity = tCapacity/tMeter->i1Divisor + 2;
            tMeter->i1Len = (INT8U)pLen;
            for (i=0; i<pLen; i++)
                tMeter->aBuf[i]=pBuf[i];
    
            nMeterUse.i2Capacity = tMeter->i2Capacity;
        }
        else
        {
            uart_Answer_Unknown();
            return;
        }
    }
    // 正常抄表
    else
    {
        gDtMeter *tMeter=(gDtMeter *)(nMeterUse.pBuf);
        
        tMeter->i1Baud = pBo;                                                                                    //  保存信息到抄表命令结构
        tMeter->i2Len = pLen;                                                                                    
        for(i=0;i<pLen;i++)
            tMeter->aBuf[i]=pBuf[i];
    }
    
    //  设置抄表超时时间
    drv_SetTimeAll(mCAC.i1ReadMeterMaxTime - 10);
    nMeterUse.fOpen = TRUE1;
}

//===============================================================================================
//  函数名称:        run_Meter
//  函数描述:   	    抄表执行函数
//  入口参数:	       
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			        陈  炎      2013-10-16      1.0          添加抄表失败分支的返回
//===============================================================================================
INT8U run_Meter()
{
    INT8U temp_buf[4],temp_len;

    if(nMeterUse.fOpen==TRUE1)
    {
        mCAC.bMeter=TRUE1;

        // 广播抄表
        if (nMeterUse.fBoard==TRUE1)
		{
            meter_broadcast();
		}
        // 正常抄表
        else if(nMeterUse.fUseYL==FALSE0 && ( nMeterUse.i1Event == RF_ASK_METER_DATA
            || nMeterUse.i1Event == RF_ASK_DLMS_DATA  || nMeterUse.i1Event == RF_ASK_TTD_DATA || nMeterUse.i1Event == RF_ASK_RRPI_DATA||  nMeterUse.i1Event==RF_ASK_CALL))
        {
            INT16U j;
            INT8U  pathNo, tFlagUp, tFlagAgain=FALSE0;
            gDtMeter *tMeter = (gDtMeter*)(nMeterUse.pBuf);
            
            if(mParse.mode_boardcast)
                mDAU[nMeterUse.i2Num].bDMode = METER_BOARD;
            
            // 单个DAU通信前初始化
            mth_WorkInit(nMeterUse.i2Num, mDAU[nMeterUse.i2Num].b4DLayerSon);

            for(j=0;j<mCAC.i2AllNum;j++)
            {
                if(dauF_In(j)==TRUE1)
                    mth_WorkSet(j);
            }

            drv_Printf("\n抄表DAU[%3d]=",nMeterUse.i2Num);
            drv_PrintfDAU(mDAU[nMeterUse.i2Num].aDAddr);

            j=30;
            if(nMeterUse.i1Event == RF_ASK_TTD_DATA)
                 j = 1;
            while(j>0)
            {
                // 抄表总时间超时
                if(drv_WaitTimeAll()==TRUE1)
                {
                    j=1;
                    if(nMeterUse.fHand==TRUE1)
                    {
                        gDtHand *tHand=(gDtHand*)(mRf.aBuf);
                        mRf.i1Com = RF_DEV_HAND;
                    
                        if(nMeterUse.i1Event == RF_ASK_METER_DATA)
                            tHand->i1Com=0x10;
                        else if(nMeterUse.i1Event == RF_ASK_CALL)
                            tHand->i1Com=0x12;
                        tHand->aBuf[0]=0x01;
                        tHand->i1Len=1;
                        
                        if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)
                        {
                            drv_RfSend(mRf.BufTx,CH_TYPE_HAND);
                        }
                    }
                    else
                    {
                        drv_Printf("\n抄表总超时时间到");
                            if(nMeterUse.i1Event !=  RF_ASK_TTD_DATA)
                        uart_Answer_Bad();
                    }
                    break;
                }
#if     MODE_UP_DOWN
                tFlagUp = mDAU[nMeterUse.i2Num].bDUpDown;
#else
                tFlagUp = FALSE0;
#endif
                if(mth_WorkRun(nMeterUse.aNumPath,tFlagUp,&pathNo)==FALSE0)
                {
                    // 路径读取失败或时间到
                    drv_Printf("\n抄表路径使用完毕");
                    if(nMeterUse.fHand==TRUE1)
                    {
                        gDtHand *tHand=(gDtHand*)(mRf.aBuf);
                        mRf.i1Com = RF_DEV_HAND;

                        if(nMeterUse.i1Event == RF_ASK_METER_DATA)
                            tHand->i1Com=0x10;
                        else if(nMeterUse.i1Event == RF_ASK_CALL)
                            tHand->i1Com=0x12;
                        tHand->aBuf[0]=0x01;
                        tHand->i1Len=1;

                        if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)
                        {
                            drv_RfSend(mRf.BufTx,CH_TYPE_HAND);
                        }
                    }
                    else
                    {
                        drv_Printf("\n测量频道抄表结束");
                          if(nMeterUse.i1Event !=  RF_ASK_TTD_DATA)
                        uart_Answer_Bad();
                    }
                    
                    break;
                }
                
                tFlagUp = FALSE0;
                // 为上下不同径
                if (pathNo == 8)
                {
                    tFlagUp = TRUE1;
                    mDAU[nMeterUse.i2Num].bDUpDown = FALSE0;
                }

                cac_2ByteTo6Byte(nMeterUse.aNumPath,nMeterUse.aPath,tFlagUp);

                tMeter->i1Path=0;
                // 有路由信道遍历
                if(BYTE_READ_BIT(mDAU[nMeterUse.i2Num].bDScanRssi, pathNo))
                {
                    tMeter->i1Path=mth_3PathLayer(nMeterUse.i2Num, pathNo);
#ifdef      MODE_UP_DOWN_CH
                    cac_CheckPathCH2(nMeterUse.i2Num, tMeter->aPathCH, pathNo);
#else
                    cac_CheckPathCH(nMeterUse.i2Num, tMeter->aPathCH, pathNo);
#endif
                }

                // 单次抄表
                if(meter_one_check(tFlagUp, tFlagAgain)==TRUE1)
                {
                    mDAU[nMeterUse.i2Num].bDMode = METER_PATH;

     
                    if (tFlagUp == FALSE0)
                        mth_WorkYes(nMeterUse.i2Num);
                    else
                        mDAU[nMeterUse.i2Num].bDUpDown = TRUE1;

                    // 串口抄表
                    if(nMeterUse.fHand==FALSE0)
                    {
                        // 抄表
                        if(nMeterUse.i1Event == RF_ASK_METER_DATA || nMeterUse.i1Event == RF_ASK_DLMS_DATA|| nMeterUse.i1Event == RF_ASK_TTD_DATA )
                        {
                            gDtMeter *tMeters=(gDtMeter *)nMeterUse.pBuf;  
                       
                            if(flag_at_meter == 1)
                            {
                              
                                   if((tMeters->aBuf != NULL)&&(tMeters->i2Len != 0))
                                  {
                                    drv_UartSend("\r",1);             
                                    NUM_2_ASCII_AT(tMeters->i2Len, temp_buf,&temp_len); // 暂且认为表计不会超过四位数
                                    drv_UartSend(temp_buf,temp_len);
                                  
                                    drv_UartSend(",",1);
                                    drv_UartSend(tMeters->aBuf,tMeters->i2Len);
                                    drv_UartSend("\r",1);
                                    
                                  }         
                          
                            }else{
                              
                                  uart_Back_DAUData(nMeterUse.aDAU,tMeters->aBuf,tMeters->i2Len,RF_ASK_METER_DATA,nMeterUse.i1XieYi);
                       
                            }

                        }
                        // 查询路径信息
                        else if(nMeterUse.i1Event == RF_ASK_RRPI_DATA)
                        {
                          
                              INT8U   path_channel, temp_len1;
                    
                              
                              path_channel    =   sReadSuccessOldPath(tmpDAUNum_RRPI);  // 读取上次成功路径，成功返回序号，失败返回13
                              

                                meterBuf[0] = '\r';
                                    Switch_Addr_6_LSB_MSB(dst_addr_RRPI);
                                HEX_2_ASCII_AT( dst_addr_RRPI, meterBuf + 1, 6 ); 
                              
                                
                                meterBuf[13] = ',';
                                mth_ReadmMath_AT( meterBuf + 14, tmpDAUNum_RRPI, path_channel, (INT8U* )&path_channel, &temp_len1 );
                                meterBuf[14 + temp_len1] = '\r';
                                drv_UartSend( meterBuf, 15 + temp_len1 );
                 
                              
                           
                        }
                    }
                    // 掌机抄表
                    else
                    {
                        INT16U i;
                        gDtHand *tHand=(gDtHand*)(mRf.aBuf);

                        mRf.i1Com = RF_DEV_HAND;
                        // 抄表
                        if(nMeterUse.i1Event == RF_ASK_METER_DATA)
                        {
                            gDtMeter *tMeters=(gDtMeter *)nMeterUse.pBuf;

                            tHand->i1Com=0x10;
                            if(FlgReceiveData)
                                tHand->aBuf[0]=0x00;	//抄表成功
                            else
                                tHand->aBuf[0]=0x01;	//抄表失败
                            tHand->aBuf[1]=nMeterUse.aDAU[0];
                            tHand->aBuf[2]=nMeterUse.aDAU[1];
                            tHand->aBuf[3]=nMeterUse.aDAU[2];
                            tHand->aBuf[4]=nMeterUse.aDAU[3];
                            tHand->aBuf[5]=nMeterUse.aDAU[4];
                            tHand->aBuf[6]=nMeterUse.aDAU[5];

                            tHand->aBuf[7]=(INT8U)tMeters->i2Len;

                            for(i=0;i<tHand->aBuf[7];i++)
                            {
                                tHand->aBuf[8+i]=tMeters->aBuf[i];
                            }

                            tHand->i1Len=8+tHand->aBuf[7];
                            
                            if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)
                            {
                                drv_RfSend(mRf.BufTx,CH_TYPE_HAND);
                            }
                        }
                        // 点名
                        else if(nMeterUse.i1Event == RF_ASK_CALL)
                        {
                            tHand->i1Com=0x12;
                            tHand->aBuf[0]=0;

                            tHand->aBuf[1]=nMeterUse.aDAU[0];
                            tHand->aBuf[2]=nMeterUse.aDAU[1];
                            tHand->aBuf[3]=nMeterUse.aDAU[2];
                            tHand->aBuf[4]=nMeterUse.aDAU[3];
                            tHand->aBuf[5]=nMeterUse.aDAU[4];
                            tHand->aBuf[6]=nMeterUse.aDAU[5];

                            tHand->aBuf[7]=0;

                            tHand->i1Len=8;
                            
                            if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)
                            {
                                drv_RfSend(mRf.BufTx,CH_TYPE_HAND);
                            }
                        }
                    }

                    mDAU[nMeterUse.i2Num].bDGood=TRUE1;

                    break;
                }
              

                j--;
            }
        }
        // 游离抄表
        else if(nMeterUse.fUseYL==TRUE1 
            && (nMeterUse.i1Event==RF_ASK_METER_DATA || nMeterUse.i1Event == RF_ASK_CALL))
        {
            INT8U j;
            INT8U flag1 = FALSE0;                                                                               //  抄表成功标志

            j=2; 
            while(j>0)
            {
                j--;
                // 单次抄表
                if(meter_one_check(FALSE0, FALSE0)==TRUE1)
                {
                     mth_WorkYes(nMeterUse.i2Num);
                    // 串口抄表
                    if(nMeterUse.fHand==FALSE0)
                    {
                        // 抄表
                        if(nMeterUse.i1Event == RF_ASK_METER_DATA)
                        {
                            gDtMeter *tMeters=(gDtMeter *)nMeterUse.pBuf;
                            flag1 = TRUE1;
                            uart_Back_DAUData(nMeterUse.aDAU,tMeters->aBuf,tMeters->i2Len,0xA6,nMeterUse.i1XieYi);
                        }
                        // 点名
                        else
                        {
                            ;
                        }
                    }

                    mDAU[nMeterUse.i2Num].bDGood=TRUE1;
                    break;
                }
            }
            if(flag1 == FALSE0)
            {
                if(nMeterUse.i1Event !=  RF_ASK_TTD_DATA)
                uart_Answer_Bad();
            }
        }

        mCAC.bMeter=FALSE0;
        nMeterUse.fOpen=FALSE0;

        return TRUE1;
    }

    return FALSE0;
}

// 单次抄表
INT8U meter_one_check(INT8U pFUp, INT8U again)
{
    INT8U tAllSend;                        // 总体
    INT8U tNowSend=0;
    INT8U tmpX;
    INT8U tFlag=FALSE0;
    INT16U tWait, i;
    INT8U tCH;
    
    // 获取本条路径本次抄表使用参数 
    mth_GetPathCH(&tCH, &tAllSend);

    while(tNowSend<tAllSend)
    {
        //==========================================
        //  构造发送数据,设置等待时间
        //==========================================
        // CAC地址、DAU地址赋值
        mRf.aCAC[0]=mCAC.aCAddr[0];
        mRf.aCAC[1]=mCAC.aCAddr[1];
        mRf.aCAC[2]=mCAC.aCAddr[2];
        mRf.aCAC[3]=mCAC.aCAddr[3];
        mRf.aCAC[4]=mCAC.aCAddr[4];
        mRf.aCAC[5]=mCAC.aCAddr[5];

        mRf.aDAU[0]=nMeterUse.aDAU[0];
        mRf.aDAU[1]=nMeterUse.aDAU[1];
        mRf.aDAU[2]=nMeterUse.aDAU[2];
        mRf.aDAU[3]=nMeterUse.aDAU[3];
        mRf.aDAU[4]=nMeterUse.aDAU[4];
        mRf.aDAU[5]=nMeterUse.aDAU[5];

        // 上下同径
        if(pFUp==FALSE0)
        {
            tmpX=cac_LayerInPath(mRf.aDAU,nMeterUse.aPath);
            if(tmpX>1 && tmpX<=MAX_LAYER)
            {
            }
            else
            {
                tmpX=1;
                tFlag=TRUE1;
            }
			tWait=meter_waitTime(tmpX);
            mRf.aPath   = nMeterUse.aPath;
            mRf.i1Layer = tmpX;
            mRf.i1UpLayer=0;
            drv_Printf("\n路径层次= %d\n抄表路径：", tmpX);
            for(i=0;i<36;i++)
            {
                drv_Printf("%.2x ", mRf.aPath[i]);
            }
        }
        // 上下不同径
        else
        {
            tFlag=TRUE1;
            mRf.i1Layer=cac_LayerInPath(mRf.aDAU,nMeterUse.aPath);
            mRf.i1UpLayer=cac_LayerInPath(mRf.aDAU,&(nMeterUse.aPath[MAX_LAYER*6]));

            if(mRf.i1Layer>1 && mRf.i1Layer<=MAX_LAYER)
            {
                // 计算等待时间
                if(mRf.i1Layer>mRf.i1UpLayer)
					tWait=meter_waitTime(mRf.i1Layer);
                else
					tWait=meter_waitTime(mRf.i1UpLayer);
            }
            else
            {
				tWait=meter_waitTime(mRf.i1UpLayer);
            }

            mRf.aPath   = nMeterUse.aPath;
        }
        
        mRf.i1Com	= nMeterUse.i1Event;

        //  频道选择
        //  掌机
        if(nMeterUse.fHand==TRUE1 && nMeterUse.i1Event == RF_ASK_CALL)
        {
            tmpX = CH_TYPE_TEST;
        }
        //  游离
        else if(nMeterUse.fUseYL==TRUE1)
        {
            tmpX = CH_TYPE_TEST;
        }
        //  正常
        else
        {
            //  使用上下不同径
            if (pFUp==TRUE1)
                tmpX = CH_TYPE_TEST_0;

            //  其他路径使用干扰最小的频道
            else
                tmpX = tCH;

        }

        // 设置发送时间
        drv_SetTimeMeter(tWait);
        
        // 构造时需要把发送的数据赋值给 mRf
        for(i=0;i<LEN_RF;i++)
        {
            mRf.aBuf[i]=nMeterUse.pBuf[i];
        }

        // 构造数据并发送
        if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)             //  构造成功
        {
            meter_send++;
            drv_RfSend(mRf.BufTx,tmpX);
        }
        drv_Printf("\n抄表呼叫DAU=");
        drv_PrintfDAU(mRf.aDAU);
        drv_PrintTime();

        // 故障地址写0xEE；
        reportDAU[0]=0xEE;
        reportDAU[1]=0xEE;
        reportDAU[2]=0xEE;
        reportDAU[3]=0xEE;
        reportDAU[4]=0xEE;
        reportDAU[5]=0xEE;
        reportDAU[6]=FALSE0;

        for(i=0;i<36;i++)
        {
            reportPath[i]=nMeterUse.aPath[i];
        }
        
        //==========================================
        //  4. 接收流程
        //==========================================
        // 是否超时
        
   if(nMeterUse.i1Event != RF_ASK_TTD_DATA)
   {

#ifdef    _LD_USE_TEST
        tFlag = TRUE1;
#endif

#ifdef     SYSTEM_VS2005
        tFlag = TRUE1;
#endif

        drv_SetTimeDAU(1);
        while(tFlag==FALSE0&&drv_WaitTimeDAU()==FALSE0)
        {
            // 查询射频串口数据
            drv_QueryUartRf();

            // 串口有数据
            if(drvUartRf.fUartOK==TRUE1)
                tsk_uart();

            // 射频有数据
      
            if(drvUartRf.fRfOK==TRUE1										// 射频收到数据
                &&tsk_rf()==TRUE1											// 数据解析有效
                &&Cb_CmpArry_stat(mRf.aTxID,nMeterUse.aPath,6)              // 为第一层节点
                &&Cb_CmpArry_stat(mRf.aDAU,nMeterUse.aDAU,6)                // 抄目标节点DAU
                &&mRf.i1Com==nMeterUse.i1Event)                             // 为抄表帧
      
            {
                tFlag=TRUE1;
                break;
            }
        }
        // 是否超时
        while(tFlag==TRUE1&&drv_WaitTimeMeter()==FALSE0)
        {
            // 抄表总时间超时
            if(drv_WaitTimeAll()==TRUE1)
                return FALSE0;

            // 查询射频串口数据
            drv_QueryUartRf();

            // 串口有数据
            if(drvUartRf.fUartOK==TRUE1)
                tsk_uart();

            // 射频有数据
            if(drvUartRf.fRfOK==TRUE1										    // 射频收到数据
                &&tsk_rf()==TRUE1)											    // 数据解析有效
            {
                //  DAU回否认帧
                if(Cb_CmpArry_stat(mRf.aDAU,nMeterUse.aDAU,6)==TRUE1		    // 是期待DAU
                    && mRf.i1Com == RF_ANSWER_NO )                              // 是否认帧
                {
                    if(nMeterUse.fHand == TRUE1)
                    {
                        INT16U i;
                        for(i=0;i<LEN_RF;i++)
                        {
                            nMeterUse.pBuf[i]=mRf.aBuf[i];
                        }
                        FlgReceiveData = FALSE0;
                        drv_Printf("\n电表层无应答");
                        drv_PrintTime();
                        return TRUE1;
                    }
                }
                     if( mRf.i1Com == RF_ASK_RRPI_DATA )                              // 是路径查询
                {
                  
     
                    FlgReceiveData = TRUE1;
                    drv_Printf("\n路径查询成功");
          
                    
                        return TRUE1;
                    
                }
                
                
                // 回抄表
                if(Cb_CmpArry_stat(mRf.aDAU,nMeterUse.aDAU,6)==TRUE1		    // 是期待DAU
                    && mRf.i1Com ==(nMeterUse.i1Event&0x7F))		            // 是期待应答帧
                {
                    INT16U i;

                    // 成功后把接受的数据赋值给 mTskDAU
                    for(i=0;i<LEN_RF;i++)
                    {
                        nMeterUse.pBuf[i]=mRf.aBuf[i];
                    }

//#if MODE_PRINT_DEBUG == 1
//                    uart_ReportDebugData(drvUartRf.aRfRxBuf,drvUartRf.i2RfRxLen, drvUartRf.i1RfCH, drvUartRf.i1RfRiss, 117);
//#endif
                    meter_succeed++;
                    // 等待DAU的再次回复
                    WaitTransmitTime();
                    FlgReceiveData = TRUE1;
                    drv_Printf("\n抄表成功");
                    drv_PrintTime();
                    return TRUE1;
                }
            }
        }
        
 
        // 故障节点处理
#ifdef MODE_REPORT
        if(reportDAU[6]==TRUE1)
        {
            reportDAU[6]=FALSE0;

            for(i=0;i<6;i++)
            {
                if(Cb_CmpArry_stat(reportDAU,&(reportPath[i*6]),6)==TRUE1)
                    break;
            }
            if(i<6)
            {
                // 记录合格节点
                cac_6ByteTo2Byte((INT16U*)reportPath,reportPath,(INT8U)i);
                mth_cheackGood((INT16U*)reportPath,(INT8U)i);

                // 记录非法节点
                cac_6ByteTo2Byte((INT16U*)(&(reportPath[(i+1)*6])),&(reportPath[(i+1)*6]),(INT8U)(4-i));
                mth_cheackBad((INT16U*)(&(reportPath[(i+1)*6])),(INT8U)(4-i));
            }
        }
#endif
        
          }
        tNowSend++;
    }

    return FALSE0;
}

//===============================================================================================
//  函数名称:       WaitTransmitTime()
//  函数描述:   	根据收到信息的中继索引值计算等待传输时间
//  入口参数:	 
//  返回值  :        
//  说明    ：  
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>             
//                             刘柱    xxxx-xx-xx            0.0          生成函数WaitTransmitTime()
//                             陈炎    2014-04-29            0.1          添加注释，更改函数名      
//===============================================================================================
void WaitTransmitTime()
{
    // 若中继索引是0，表示是第一层节点发出的
	if(mRf.i1NoteID==0)
		return;

    // 若中继索引是1~6，表示是第2-7层节点发出
	if(mRf.i1NoteID < MAX_LAYER)
	{
		// 设置发送时间
		drv_SetTimeDAU(mRf.i1NoteID);

		// 等待超时，此过程仅响应立即响应类命令
		while(drv_WaitTimeDAU()==FALSE0)
		{
			// 查询射频串口数据
			drv_QueryUartRf();

			// 串口有数据
			if(drvUartRf.fUartOK==TRUE1)
				tsk_uart();

			// 射频有数据不处理
			if(drvUartRf.fRfOK==TRUE1)	
				tsk_rf();
		}
	}
}

// 抄表时间等待
INT8U meter_waitTime(INT8U pLayer)
{
    // 上行
    switch(pLayer)
    {
    case 1:
        return 10;
    case 2:
        return 11;
    case 3 :
        return 12;
    case 4:
        return 13;
    case 5:
        return 14;
    case 6:
        return 16;
    case 7:
        return 18;
    default:
        return 10;
    }
}

INT16U meter_waitTime1(INT8U pLayer, INT16U xLen)
{
    // 串口速率 2400bps，1B需要（11/2.4）ms
    // 空中速率 10Kbps ，1B需8/10K = 0.8ms，最小影响长度50/0.8 = 63B
    // 设数645数据长为x，层次为n,电表延时500ms，每次k = 50ms补偿;
    // 物理层固定长度 = 80(前导) + 2（分隔符）+ 4(物理头) + 2（校验序列）= 88B
    // MAC 层固定长度 = 2(MAC帧控制域) + 1(序号) + 2(PanID) + 12(地址) = 17（B）
    // NET 层固定长度 = 1(NET帧控制域) + 12(地址) + 6*(pLayer-1)(路由信息域) = 7 + 6*pLayer
    // APS 层固定长度 = 1(APS帧控制域) + 1(序号) + 3(数据区头)               =  5
    // 下行串口=x; 下行空中= 117+6n+x近似于（160+x）; 上行空中=(88-3)+255=340; 上行串口=255
    // 总延时 t = (160+x + 340)*0.8*n + (255+x)*11/2.4 + 500 + 50n
    //          = 0.8*n*x + 450n + 1669 + 4.5x = 50*(9n+34)+(0.8n + 4.5)x

    return (INT16U)((pLayer*8/5 + 9)*xLen/100 + 9*pLayer + 34);
}

// 选择抄表频道，xTime表次数取值范围0~3  --  定义与meter.c
INT8U meter_ChooseCH(INT8U xTime)
{
    if(xTime == 0)
        sState = 1;

    switch(sState)
    {
    case 1:
        {
            sState = 2;
            // 工作0干扰小于工作1
            if(mCAC.aCBGNoise[2]>mCAC.aCBGNoise[3])
            {
                if((mCAC.aCBGNoise[2] - mCAC.aCBGNoise[3])>MIN_NOISE)
                    sState = 3;
                return CH_TYPE_WORK_0;
            }
            else
            {
                if((mCAC.aCBGNoise[3] - mCAC.aCBGNoise[2])>MIN_NOISE)
                    sState = 3;
                return CH_TYPE_WORK_1;
            }
        }
    case 2:
        {
            sState = 4;
            if(mCAC.aCBGNoise[2]>mCAC.aCBGNoise[3])
                return CH_TYPE_WORK_1;
            else
                return CH_TYPE_WORK_0;
        }
    case 3:
        {
            sState = 4;
            return CH_TYPE_TEST_1;
        }
    case 4:
        {
            sState = 5;
            return CH_TYPE_TEST_0;
        }
    default:
        return 0xFF;
    }
}

INT8U   meter_ChooseCH3(INT16U *pPath, INT8U pLayer)
{
    INT8U i, j;
    INT8U channel[4], bestchannel;
    INT16U tN, noise, bestnoise=0;
    
    channel[0] = 0;
    channel[1] = 1;
    channel[2] = mCAC.i1BigCH<<1;
    channel[3] = 1+(mCAC.i1BigCH<<1);
    
    // 计算四个频道的路径底噪信息, 如果节点不属于本网是没有底噪信息的
    for(i=0; i<4; i++)
    {
        noise = 0;
        for(j=0; j<pLayer-1; j++)
        {
            tN = nMeterUse.aNumPath[j];
            noise += mDAU[tN].aDBGNoise[channel[i]];
        }
        noise += mCAC.aCBGNoise[i];
        noise += mDAU[nMeterUse.i2Num].aDBGNoise[channel[i]];

        if (noise>=bestnoise)
        {
            bestnoise = noise;
            bestchannel = i;
        }
    }

    return CH_TYPE_TEST_0 + bestchannel;
}

void meter_broadcast()
{
    INT16U i;
    INT8U tFlag=FALSE0, tmpX = CH_TYPE_WORK_1;
    gDtBroadMeter *tMeters=(gDtBroadMeter *)nMeterUse.pBuf;

    if (dauF_In(nMeterUse.i2Num)==TRUE1)
    {
        mDAU[nMeterUse.i2Num].bDMode = METER_PATH;
    }

    Co_CpyArry_stat(nMeterUse.aDAU, mRf.aDAU, 6);
    Co_CpyArry_stat(mCAC.aCAddr, mRf.aCAC, 6);
    mRf.i1Com = RF_BROADCAST_METER;
    mRf.i1Layer = 7;
    for (i=0; i<LEN_RF; i++)
    {
        mRf.aBuf[i] = nMeterUse.pBuf[i];
    }

    // 构造数据并发送
    if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)             //  构造成功
    {
        drv_RfSend(mRf.BufTx, tmpX);
    }

    drv_SetTimeBroad(3+(nMeterUse.i2Capacity*3/5));
    while(drv_WaitTimeBroad()==FALSE0)
    {
        // 查询射频串口数据
        drv_QueryUartRf();

        // 串口有数据
        if(drvUartRf.fUartOK==TRUE1)
            tsk_uart();

        // 射频有数据
        if (drvUartRf.fRfOK==TRUE1 && tsk_rf()==TRUE1 && tFlag==FALSE0
            && Cb_CmpArry_stat(mRf.aDAU, nMeterUse.aDAU, 6)==TRUE1
            && mRf.i1Com==RF_BROADCAST_METER)
        {
            // 成功后把接受的数据赋值给 mTskDAU
            for(i=0;i<LEN_RF;i++)
                nMeterUse.pBuf[i]=mRf.aBuf[i];

            if (tMeters->i1BroadNO==(INT8U)mCAC.i2BrdNum)
                tFlag=TRUE1;
        }
        
        // 抄表总时间到
        if(drv_WaitTimeAll()==TRUE1)
            break;
    }

    mCAC.i2BrdNum += 2;
    // 抄表成功
    if (tFlag)
    {
        uart_Back_DAUData(nMeterUse.aDAU,tMeters->aBuf,tMeters->i1Len,RF_ASK_METER_DATA,nMeterUse.i1XieYi);
    }
    // 抄表失败
    else
    {
        // 直接使用广播抄表的可再次广播抄表
        if (0)//nMeterUse.fBoard==TRUE1)
        {
            gDtBroadMeter *tMeter = (gDtBroadMeter*)nMeterUse.pBuf;
            INT16U pNum = nMeterUse.i2Num;
            INT16U tCapacity = dauF_In(pNum)?mDAU[pNum].b10DShiXi:mCAC.i2DownDAU;
        
            if ((mCAC.i2BrdNum&0x00FF)<2)
                mCAC.i2BrdNum |= 0x0002;
            tMeter->i1BroadNO = (INT8U)mCAC.i2BrdNum++;
            tMeter->i2Shixi   = 0;
            tMeter->i1Divisor = meter_board_divisor(tCapacity, 30)+1;
            tMeter->i2Capacity = tCapacity/tMeter->i1Divisor + 2;
            tMeter->i1Len = (INT8U)nMeterUse.i2Len;
            for (i=0; i<tMeter->i1Len; i++)
                tMeter->aBuf[i]=meterBuf[i];
            
            nMeterUse.i2Capacity = tMeter->i2Capacity;
            drv_Printf("\n广播抄表失败, 当前可进行二次广播");
            meter_broadcast_again();
        }
        else
        {
            drv_Printf("\n广播抄表失败, 当前不可进行二次广播");
            if(mDAU[nMeterUse.i2Num].b2DMeter<3)
                mDAU[nMeterUse.i2Num].b2DMeter++;
            if(mDAU[nMeterUse.i2Num].b3Divisor<4)
                mDAU[nMeterUse.i2Num].b3Divisor++;
            else
                mDAU[nMeterUse.i2Num].b3Divisor=0;
            uart_Answer_Bad();
        }
    }
}

void meter_broadcast_again()
{
    INT16U i;
    INT8U tFlag=FALSE0, tmpX = CH_TYPE_TEST_1;
    gDtBroadMeter *tMeters=(gDtBroadMeter *)nMeterUse.pBuf;

    if (dauF_In(nMeterUse.i2Num)==TRUE1)
    {
        mDAU[nMeterUse.i2Num].bDMode = METER_PATH;
        tmpX = CH_TYPE_WORK_1;
    }

    Co_CpyArry_stat(nMeterUse.aDAU, mRf.aDAU, 6);
    Co_CpyArry_stat(mCAC.aCAddr, mRf.aCAC, 6);
    mRf.i1Com = RF_BROADCAST_METER;
    mRf.i1Layer = 7;
    for (i=0; i<LEN_RF; i++)
    {
        mRf.aBuf[i] = nMeterUse.pBuf[i];
    }

    // 构造数据并发送
    if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)             //  构造成功
    {
        drv_RfSend(mRf.BufTx, tmpX);
    }

    drv_SetTimeBroad(1+(nMeterUse.i2Capacity*3/5));
    while(drv_WaitTimeBroad()==FALSE0)
    {
        // 查询射频串口数据
        drv_QueryUartRf();

        // 串口有数据
        if(drvUartRf.fUartOK==TRUE1)
            tsk_uart();

        // 射频有数据
        if (drvUartRf.fRfOK==TRUE1 && tsk_rf()==TRUE1 && tFlag==FALSE0
            && Cb_CmpArry_stat(mRf.aDAU, nMeterUse.aDAU, 6)==TRUE1
            && mRf.i1Com==RF_BROADCAST_METER)
        {
            // 成功后把接受的数据赋值给 mTskDAU
            for(i=0;i<LEN_RF;i++)
                nMeterUse.pBuf[i]=mRf.aBuf[i];

            if (tMeters->i1BroadNO==(INT8U)mCAC.i2BrdNum)
                tFlag=TRUE1;
        }
    }

    mCAC.i2BrdNum += 2;
    // 抄表成功
    if (tFlag)
    {
        drv_Printf("\n广播抄表成功");
        uart_Back_DAUData(nMeterUse.aDAU,tMeters->aBuf,tMeters->i1Len,RF_ASK_METER_DATA,nMeterUse.i1XieYi);
    }
    // 抄表失败
    else
    {
        drv_Printf("\n广播抄表失败");
        if(mDAU[nMeterUse.i2Num].b2DMeter<3)
            mDAU[nMeterUse.i2Num].b2DMeter++;
        uart_Answer_Bad();
    }
}

// 压缩因子
INT8U meter_board_divisor(INT16U pCapacity, INT8U pTime)
{
    return 1+((3*pCapacity)/(5*pTime));
}
