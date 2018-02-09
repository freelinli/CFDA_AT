#include	"prio_II.h"
#include        "uart_2013_AT.h"

#if PRO_UART == 2013
extern INT8U gi1UpSeqNum;                                                                       // ��ʱ��Ӧ�౨�����
extern INT8U gi1DownSeqNum;                                                                     // ���б������
extern INT8U gi1ReportSeqNum;                                                                   // �ϱ��������
#endif

#define         MIN_NOISE          7                                                            // �����������
static INT8U    sState;                                                                         // ����Ƶ��ѡ��
INT32U          meter_send;
INT32U          meter_succeed;
INT8U		    reportDAU[7];                                                                   // �����ϱ��ڵ��¼
INT8U			reportPath[6*MAX_LAYER];                                                        // �ϱ�·��
INT8U			FlgReceiveData;                                                                 // ����Ӧ���־ 

INT8U           meter_ChooseCH              (INT8U xTime);                                      // ����Ƶ��ѡ�� 
INT8U           meter_ChooseCH3             (INT16U *pPath, INT8U pLayer);
INT8U			meter_one_check				(INT8U pF, INT8U again);						                    // ���γ���
void			WaitTransmitTime		    (void);							                    // �ȴ�����ʱ��
INT8U			meter_waitTime				(INT8U pLayer);		                // ����ʱ��ȴ�
void            meter_broadcast             (void);
void            meter_broadcast_again       (void);
INT8U           meter_board_divisor         (INT16U pCapacity, INT8U pTime);

INT8U           meterBuf[LEN_USER];

extern INT8U   flag_at_meter;


gMeterUse nMeterUse;						// ��������ṹ 


INT8U tmpDAUNum_RRPI = 0;
INT8U dst_addr_RRPI[6];

// �����ʼ��
void init_Meter()
{
    //  ���ó���ģʽ
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

    //  ��ճ���ṹ��
    Co_SetArry_stat((INT8U*)(&nMeterUse),0,sizeof(gMeterUse));
    nMeterUse.pBuf  = drvBuf.pmMeter;
}

// �ƻ�������������
void Hand_set_Meter(INT16U pNum,INT8U pCom,INT8U *pBuf,INT16U pLen)
{
    INT16U i;
    gDtMeter *tMeter=(gDtMeter *)(nMeterUse.pBuf);

    // ��������
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
//  ��������:        set_Meter
//  ��������:   	    ������������
//  ��ڲ���:	 
//
//             1.  pAddr    <����>  INT8U*
//                                 <˵��>  �ַ���ָ�룬ָ��ӽڵ��ַ
//             2.  pBuf       <����>  INT8U*
//                                 <˵��>  �ַ���ָ�룬ָ��645֡
//             3.  pLen       <����>  INT16U
//                                 <˵��>  �ַ�������
//             4.  pXie       <����>  INT8U
//                                 <˵��>  Э�����ͣ���δʹ�� 
//             5.  pBo        <����>  INT8U
//                                 <˵��>  ������
//          
//  ����ֵ  :        
//  ˵��    :        
//  �޸ļ�¼:  <����>      <�޸�����>     <�汾 >      <����>       
//			                 �� ��    2013-09-25           1.0           ���ɺ���
//                           �� ��    2013-12-10           1.0           �޸�ע��
//===============================================================================================
void set_Meter(INT16U pNum, INT8U *pBuf, INT16U pLen, INT8U pXie, INT8U pBo, INT8U pCom)
{
    INT16U i;
  
    nMeterUse.aDAU[0]=mDAU[pNum].aDAddr[0];                                          //  ������Ϣ����������ṹ
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
    
    // ����ģʽΪ�㲥, �㲥ʧ�ܴ�������3��
    if (mDAU[pNum].bDMode==METER_BOARD && mDAU[pNum].b2DMeter<3)
        nMeterUse.fBoard = TRUE1;
    // ����ģʽΪ�㲥, �㲥ʧ�ܴ���Ϊ3��
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
    gi1UpSeqNum = gi1DownSeqNum;                                                              //  ȷ�ϻ᳭����޸����б������
#endif
    
    // ���ݳ�������
    for(i=0; i<pLen; i++)
        meterBuf[i] = pBuf[i];
    
    // �㲥����
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
    // ��������
    else
    {
        gDtMeter *tMeter=(gDtMeter *)(nMeterUse.pBuf);
        
        tMeter->i1Baud = pBo;                                                                                    //  ������Ϣ����������ṹ
        tMeter->i2Len = pLen;                                                                                    
        for(i=0;i<pLen;i++)
            tMeter->aBuf[i]=pBuf[i];
    }
    
    //  ���ó���ʱʱ��
    drv_SetTimeAll(mCAC.i1ReadMeterMaxTime - 10);
    nMeterUse.fOpen = TRUE1;
}

//===============================================================================================
//  ��������:        run_Meter
//  ��������:   	    ����ִ�к���
//  ��ڲ���:	       
//  ����ֵ  :        
//  ˵��    :        
//  �޸ļ�¼:  <����>      <�޸�����>     <�汾 >      <����>       
//			        ��  ��      2013-10-16      1.0          ��ӳ���ʧ�ܷ�֧�ķ���
//===============================================================================================
INT8U run_Meter()
{
    INT8U temp_buf[4],temp_len;

    if(nMeterUse.fOpen==TRUE1)
    {
        mCAC.bMeter=TRUE1;

        // �㲥����
        if (nMeterUse.fBoard==TRUE1)
		{
            meter_broadcast();
		}
        // ��������
        else if(nMeterUse.fUseYL==FALSE0 && ( nMeterUse.i1Event == RF_ASK_METER_DATA
            || nMeterUse.i1Event == RF_ASK_DLMS_DATA  || nMeterUse.i1Event == RF_ASK_TTD_DATA || nMeterUse.i1Event == RF_ASK_RRPI_DATA||  nMeterUse.i1Event==RF_ASK_CALL))
        {
            INT16U j;
            INT8U  pathNo, tFlagUp, tFlagAgain=FALSE0;
            gDtMeter *tMeter = (gDtMeter*)(nMeterUse.pBuf);
            
            if(mParse.mode_boardcast)
                mDAU[nMeterUse.i2Num].bDMode = METER_BOARD;
            
            // ����DAUͨ��ǰ��ʼ��
            mth_WorkInit(nMeterUse.i2Num, mDAU[nMeterUse.i2Num].b4DLayerSon);

            for(j=0;j<mCAC.i2AllNum;j++)
            {
                if(dauF_In(j)==TRUE1)
                    mth_WorkSet(j);
            }

            drv_Printf("\n����DAU[%3d]=",nMeterUse.i2Num);
            drv_PrintfDAU(mDAU[nMeterUse.i2Num].aDAddr);

            j=30;
            if(nMeterUse.i1Event == RF_ASK_TTD_DATA)
                 j = 1;
            while(j>0)
            {
                // ������ʱ�䳬ʱ
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
                        drv_Printf("\n�����ܳ�ʱʱ�䵽");
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
                    // ·����ȡʧ�ܻ�ʱ�䵽
                    drv_Printf("\n����·��ʹ�����");
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
                        drv_Printf("\n����Ƶ���������");
                          if(nMeterUse.i1Event !=  RF_ASK_TTD_DATA)
                        uart_Answer_Bad();
                    }
                    
                    break;
                }
                
                tFlagUp = FALSE0;
                // Ϊ���²�ͬ��
                if (pathNo == 8)
                {
                    tFlagUp = TRUE1;
                    mDAU[nMeterUse.i2Num].bDUpDown = FALSE0;
                }

                cac_2ByteTo6Byte(nMeterUse.aNumPath,nMeterUse.aPath,tFlagUp);

                tMeter->i1Path=0;
                // ��·���ŵ�����
                if(BYTE_READ_BIT(mDAU[nMeterUse.i2Num].bDScanRssi, pathNo))
                {
                    tMeter->i1Path=mth_3PathLayer(nMeterUse.i2Num, pathNo);
#ifdef      MODE_UP_DOWN_CH
                    cac_CheckPathCH2(nMeterUse.i2Num, tMeter->aPathCH, pathNo);
#else
                    cac_CheckPathCH(nMeterUse.i2Num, tMeter->aPathCH, pathNo);
#endif
                }

                // ���γ���
                if(meter_one_check(tFlagUp, tFlagAgain)==TRUE1)
                {
                    mDAU[nMeterUse.i2Num].bDMode = METER_PATH;

     
                    if (tFlagUp == FALSE0)
                        mth_WorkYes(nMeterUse.i2Num);
                    else
                        mDAU[nMeterUse.i2Num].bDUpDown = TRUE1;

                    // ���ڳ���
                    if(nMeterUse.fHand==FALSE0)
                    {
                        // ����
                        if(nMeterUse.i1Event == RF_ASK_METER_DATA || nMeterUse.i1Event == RF_ASK_DLMS_DATA|| nMeterUse.i1Event == RF_ASK_TTD_DATA )
                        {
                            gDtMeter *tMeters=(gDtMeter *)nMeterUse.pBuf;  
                       
                            if(flag_at_meter == 1)
                            {
                              
                                   if((tMeters->aBuf != NULL)&&(tMeters->i2Len != 0))
                                  {
                                    drv_UartSend("\r",1);             
                                    NUM_2_ASCII_AT(tMeters->i2Len, temp_buf,&temp_len); // ������Ϊ��Ʋ��ᳬ����λ��
                                    drv_UartSend(temp_buf,temp_len);
                                  
                                    drv_UartSend(",",1);
                                    drv_UartSend(tMeters->aBuf,tMeters->i2Len);
                                    drv_UartSend("\r",1);
                                    
                                  }         
                          
                            }else{
                              
                                  uart_Back_DAUData(nMeterUse.aDAU,tMeters->aBuf,tMeters->i2Len,RF_ASK_METER_DATA,nMeterUse.i1XieYi);
                       
                            }

                        }
                        // ��ѯ·����Ϣ
                        else if(nMeterUse.i1Event == RF_ASK_RRPI_DATA)
                        {
                          
                              INT8U   path_channel, temp_len1;
                    
                              
                              path_channel    =   sReadSuccessOldPath(tmpDAUNum_RRPI);  // ��ȡ�ϴγɹ�·�����ɹ�������ţ�ʧ�ܷ���13
                              

                                meterBuf[0] = '\r';
                                    Switch_Addr_6_LSB_MSB(dst_addr_RRPI);
                                HEX_2_ASCII_AT( dst_addr_RRPI, meterBuf + 1, 6 ); 
                              
                                
                                meterBuf[13] = ',';
                                mth_ReadmMath_AT( meterBuf + 14, tmpDAUNum_RRPI, path_channel, (INT8U* )&path_channel, &temp_len1 );
                                meterBuf[14 + temp_len1] = '\r';
                                drv_UartSend( meterBuf, 15 + temp_len1 );
                 
                              
                           
                        }
                    }
                    // �ƻ�����
                    else
                    {
                        INT16U i;
                        gDtHand *tHand=(gDtHand*)(mRf.aBuf);

                        mRf.i1Com = RF_DEV_HAND;
                        // ����
                        if(nMeterUse.i1Event == RF_ASK_METER_DATA)
                        {
                            gDtMeter *tMeters=(gDtMeter *)nMeterUse.pBuf;

                            tHand->i1Com=0x10;
                            if(FlgReceiveData)
                                tHand->aBuf[0]=0x00;	//����ɹ�
                            else
                                tHand->aBuf[0]=0x01;	//����ʧ��
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
                        // ����
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
        // ���볭��
        else if(nMeterUse.fUseYL==TRUE1 
            && (nMeterUse.i1Event==RF_ASK_METER_DATA || nMeterUse.i1Event == RF_ASK_CALL))
        {
            INT8U j;
            INT8U flag1 = FALSE0;                                                                               //  ����ɹ���־

            j=2; 
            while(j>0)
            {
                j--;
                // ���γ���
                if(meter_one_check(FALSE0, FALSE0)==TRUE1)
                {
                     mth_WorkYes(nMeterUse.i2Num);
                    // ���ڳ���
                    if(nMeterUse.fHand==FALSE0)
                    {
                        // ����
                        if(nMeterUse.i1Event == RF_ASK_METER_DATA)
                        {
                            gDtMeter *tMeters=(gDtMeter *)nMeterUse.pBuf;
                            flag1 = TRUE1;
                            uart_Back_DAUData(nMeterUse.aDAU,tMeters->aBuf,tMeters->i2Len,0xA6,nMeterUse.i1XieYi);
                        }
                        // ����
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

// ���γ���
INT8U meter_one_check(INT8U pFUp, INT8U again)
{
    INT8U tAllSend;                        // ����
    INT8U tNowSend=0;
    INT8U tmpX;
    INT8U tFlag=FALSE0;
    INT16U tWait, i;
    INT8U tCH;
    
    // ��ȡ����·�����γ���ʹ�ò��� 
    mth_GetPathCH(&tCH, &tAllSend);

    while(tNowSend<tAllSend)
    {
        //==========================================
        //  ���췢������,���õȴ�ʱ��
        //==========================================
        // CAC��ַ��DAU��ַ��ֵ
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

        // ����ͬ��
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
            drv_Printf("\n·�����= %d\n����·����", tmpX);
            for(i=0;i<36;i++)
            {
                drv_Printf("%.2x ", mRf.aPath[i]);
            }
        }
        // ���²�ͬ��
        else
        {
            tFlag=TRUE1;
            mRf.i1Layer=cac_LayerInPath(mRf.aDAU,nMeterUse.aPath);
            mRf.i1UpLayer=cac_LayerInPath(mRf.aDAU,&(nMeterUse.aPath[MAX_LAYER*6]));

            if(mRf.i1Layer>1 && mRf.i1Layer<=MAX_LAYER)
            {
                // ����ȴ�ʱ��
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

        //  Ƶ��ѡ��
        //  �ƻ�
        if(nMeterUse.fHand==TRUE1 && nMeterUse.i1Event == RF_ASK_CALL)
        {
            tmpX = CH_TYPE_TEST;
        }
        //  ����
        else if(nMeterUse.fUseYL==TRUE1)
        {
            tmpX = CH_TYPE_TEST;
        }
        //  ����
        else
        {
            //  ʹ�����²�ͬ��
            if (pFUp==TRUE1)
                tmpX = CH_TYPE_TEST_0;

            //  ����·��ʹ�ø�����С��Ƶ��
            else
                tmpX = tCH;

        }

        // ���÷���ʱ��
        drv_SetTimeMeter(tWait);
        
        // ����ʱ��Ҫ�ѷ��͵����ݸ�ֵ�� mRf
        for(i=0;i<LEN_RF;i++)
        {
            mRf.aBuf[i]=nMeterUse.pBuf[i];
        }

        // �������ݲ�����
        if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)             //  ����ɹ�
        {
            meter_send++;
            drv_RfSend(mRf.BufTx,tmpX);
        }
        drv_Printf("\n�������DAU=");
        drv_PrintfDAU(mRf.aDAU);
        drv_PrintTime();

        // ���ϵ�ַд0xEE��
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
        //  4. ��������
        //==========================================
        // �Ƿ�ʱ
        
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
            // ��ѯ��Ƶ��������
            drv_QueryUartRf();

            // ����������
            if(drvUartRf.fUartOK==TRUE1)
                tsk_uart();

            // ��Ƶ������
      
            if(drvUartRf.fRfOK==TRUE1										// ��Ƶ�յ�����
                &&tsk_rf()==TRUE1											// ���ݽ�����Ч
                &&Cb_CmpArry_stat(mRf.aTxID,nMeterUse.aPath,6)              // Ϊ��һ��ڵ�
                &&Cb_CmpArry_stat(mRf.aDAU,nMeterUse.aDAU,6)                // ��Ŀ��ڵ�DAU
                &&mRf.i1Com==nMeterUse.i1Event)                             // Ϊ����֡
      
            {
                tFlag=TRUE1;
                break;
            }
        }
        // �Ƿ�ʱ
        while(tFlag==TRUE1&&drv_WaitTimeMeter()==FALSE0)
        {
            // ������ʱ�䳬ʱ
            if(drv_WaitTimeAll()==TRUE1)
                return FALSE0;

            // ��ѯ��Ƶ��������
            drv_QueryUartRf();

            // ����������
            if(drvUartRf.fUartOK==TRUE1)
                tsk_uart();

            // ��Ƶ������
            if(drvUartRf.fRfOK==TRUE1										    // ��Ƶ�յ�����
                &&tsk_rf()==TRUE1)											    // ���ݽ�����Ч
            {
                //  DAU�ط���֡
                if(Cb_CmpArry_stat(mRf.aDAU,nMeterUse.aDAU,6)==TRUE1		    // ���ڴ�DAU
                    && mRf.i1Com == RF_ANSWER_NO )                              // �Ƿ���֡
                {
                    if(nMeterUse.fHand == TRUE1)
                    {
                        INT16U i;
                        for(i=0;i<LEN_RF;i++)
                        {
                            nMeterUse.pBuf[i]=mRf.aBuf[i];
                        }
                        FlgReceiveData = FALSE0;
                        drv_Printf("\n������Ӧ��");
                        drv_PrintTime();
                        return TRUE1;
                    }
                }
                     if( mRf.i1Com == RF_ASK_RRPI_DATA )                              // ��·����ѯ
                {
                  
     
                    FlgReceiveData = TRUE1;
                    drv_Printf("\n·����ѯ�ɹ�");
          
                    
                        return TRUE1;
                    
                }
                
                
                // �س���
                if(Cb_CmpArry_stat(mRf.aDAU,nMeterUse.aDAU,6)==TRUE1		    // ���ڴ�DAU
                    && mRf.i1Com ==(nMeterUse.i1Event&0x7F))		            // ���ڴ�Ӧ��֡
                {
                    INT16U i;

                    // �ɹ���ѽ��ܵ����ݸ�ֵ�� mTskDAU
                    for(i=0;i<LEN_RF;i++)
                    {
                        nMeterUse.pBuf[i]=mRf.aBuf[i];
                    }

//#if MODE_PRINT_DEBUG == 1
//                    uart_ReportDebugData(drvUartRf.aRfRxBuf,drvUartRf.i2RfRxLen, drvUartRf.i1RfCH, drvUartRf.i1RfRiss, 117);
//#endif
                    meter_succeed++;
                    // �ȴ�DAU���ٴλظ�
                    WaitTransmitTime();
                    FlgReceiveData = TRUE1;
                    drv_Printf("\n����ɹ�");
                    drv_PrintTime();
                    return TRUE1;
                }
            }
        }
        
 
        // ���Ͻڵ㴦��
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
                // ��¼�ϸ�ڵ�
                cac_6ByteTo2Byte((INT16U*)reportPath,reportPath,(INT8U)i);
                mth_cheackGood((INT16U*)reportPath,(INT8U)i);

                // ��¼�Ƿ��ڵ�
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
//  ��������:       WaitTransmitTime()
//  ��������:   	�����յ���Ϣ���м�����ֵ����ȴ�����ʱ��
//  ��ڲ���:	 
//  ����ֵ  :        
//  ˵��    ��  
//  �޸ļ�¼:  <����>      <�޸�����>     <�汾 >      <����>             
//                             ����    xxxx-xx-xx            0.0          ���ɺ���WaitTransmitTime()
//                             ����    2014-04-29            0.1          ���ע�ͣ����ĺ�����      
//===============================================================================================
void WaitTransmitTime()
{
    // ���м�������0����ʾ�ǵ�һ��ڵ㷢����
	if(mRf.i1NoteID==0)
		return;

    // ���м�������1~6����ʾ�ǵ�2-7��ڵ㷢��
	if(mRf.i1NoteID < MAX_LAYER)
	{
		// ���÷���ʱ��
		drv_SetTimeDAU(mRf.i1NoteID);

		// �ȴ���ʱ���˹��̽���Ӧ������Ӧ������
		while(drv_WaitTimeDAU()==FALSE0)
		{
			// ��ѯ��Ƶ��������
			drv_QueryUartRf();

			// ����������
			if(drvUartRf.fUartOK==TRUE1)
				tsk_uart();

			// ��Ƶ�����ݲ�����
			if(drvUartRf.fRfOK==TRUE1)	
				tsk_rf();
		}
	}
}

// ����ʱ��ȴ�
INT8U meter_waitTime(INT8U pLayer)
{
    // ����
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
    // �������� 2400bps��1B��Ҫ��11/2.4��ms
    // �������� 10Kbps ��1B��8/10K = 0.8ms����СӰ�쳤��50/0.8 = 63B
    // ����645���ݳ�Ϊx�����Ϊn,�����ʱ500ms��ÿ��k = 50ms����;
    // �����̶����� = 80(ǰ��) + 2���ָ�����+ 4(����ͷ) + 2��У�����У�= 88B
    // MAC ��̶����� = 2(MAC֡������) + 1(���) + 2(PanID) + 12(��ַ) = 17��B��
    // NET ��̶����� = 1(NET֡������) + 12(��ַ) + 6*(pLayer-1)(·����Ϣ��) = 7 + 6*pLayer
    // APS ��̶����� = 1(APS֡������) + 1(���) + 3(������ͷ)               =  5
    // ���д���=x; ���п���= 117+6n+x�����ڣ�160+x��; ���п���=(88-3)+255=340; ���д���=255
    // ����ʱ t = (160+x + 340)*0.8*n + (255+x)*11/2.4 + 500 + 50n
    //          = 0.8*n*x + 450n + 1669 + 4.5x = 50*(9n+34)+(0.8n + 4.5)x

    return (INT16U)((pLayer*8/5 + 9)*xLen/100 + 9*pLayer + 34);
}

// ѡ�񳭱�Ƶ����xTime�����ȡֵ��Χ0~3  --  ������meter.c
INT8U meter_ChooseCH(INT8U xTime)
{
    if(xTime == 0)
        sState = 1;

    switch(sState)
    {
    case 1:
        {
            sState = 2;
            // ����0����С�ڹ���1
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
    
    // �����ĸ�Ƶ����·��������Ϣ, ����ڵ㲻���ڱ�����û�е�����Ϣ��
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

    // �������ݲ�����
    if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)             //  ����ɹ�
    {
        drv_RfSend(mRf.BufTx, tmpX);
    }

    drv_SetTimeBroad(3+(nMeterUse.i2Capacity*3/5));
    while(drv_WaitTimeBroad()==FALSE0)
    {
        // ��ѯ��Ƶ��������
        drv_QueryUartRf();

        // ����������
        if(drvUartRf.fUartOK==TRUE1)
            tsk_uart();

        // ��Ƶ������
        if (drvUartRf.fRfOK==TRUE1 && tsk_rf()==TRUE1 && tFlag==FALSE0
            && Cb_CmpArry_stat(mRf.aDAU, nMeterUse.aDAU, 6)==TRUE1
            && mRf.i1Com==RF_BROADCAST_METER)
        {
            // �ɹ���ѽ��ܵ����ݸ�ֵ�� mTskDAU
            for(i=0;i<LEN_RF;i++)
                nMeterUse.pBuf[i]=mRf.aBuf[i];

            if (tMeters->i1BroadNO==(INT8U)mCAC.i2BrdNum)
                tFlag=TRUE1;
        }
        
        // ������ʱ�䵽
        if(drv_WaitTimeAll()==TRUE1)
            break;
    }

    mCAC.i2BrdNum += 2;
    // ����ɹ�
    if (tFlag)
    {
        uart_Back_DAUData(nMeterUse.aDAU,tMeters->aBuf,tMeters->i1Len,RF_ASK_METER_DATA,nMeterUse.i1XieYi);
    }
    // ����ʧ��
    else
    {
        // ֱ��ʹ�ù㲥����Ŀ��ٴι㲥����
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
            drv_Printf("\n�㲥����ʧ��, ��ǰ�ɽ��ж��ι㲥");
            meter_broadcast_again();
        }
        else
        {
            drv_Printf("\n�㲥����ʧ��, ��ǰ���ɽ��ж��ι㲥");
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

    // �������ݲ�����
    if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)             //  ����ɹ�
    {
        drv_RfSend(mRf.BufTx, tmpX);
    }

    drv_SetTimeBroad(1+(nMeterUse.i2Capacity*3/5));
    while(drv_WaitTimeBroad()==FALSE0)
    {
        // ��ѯ��Ƶ��������
        drv_QueryUartRf();

        // ����������
        if(drvUartRf.fUartOK==TRUE1)
            tsk_uart();

        // ��Ƶ������
        if (drvUartRf.fRfOK==TRUE1 && tsk_rf()==TRUE1 && tFlag==FALSE0
            && Cb_CmpArry_stat(mRf.aDAU, nMeterUse.aDAU, 6)==TRUE1
            && mRf.i1Com==RF_BROADCAST_METER)
        {
            // �ɹ���ѽ��ܵ����ݸ�ֵ�� mTskDAU
            for(i=0;i<LEN_RF;i++)
                nMeterUse.pBuf[i]=mRf.aBuf[i];

            if (tMeters->i1BroadNO==(INT8U)mCAC.i2BrdNum)
                tFlag=TRUE1;
        }
    }

    mCAC.i2BrdNum += 2;
    // ����ɹ�
    if (tFlag)
    {
        drv_Printf("\n�㲥����ɹ�");
        uart_Back_DAUData(nMeterUse.aDAU,tMeters->aBuf,tMeters->i1Len,RF_ASK_METER_DATA,nMeterUse.i1XieYi);
    }
    // ����ʧ��
    else
    {
        drv_Printf("\n�㲥����ʧ��");
        if(mDAU[nMeterUse.i2Num].b2DMeter<3)
            mDAU[nMeterUse.i2Num].b2DMeter++;
        uart_Answer_Bad();
    }
}

// ѹ������
INT8U meter_board_divisor(INT16U pCapacity, INT8U pTime)
{
    return 1+((3*pCapacity)/(5*pTime));
}
