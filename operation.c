#include	"prio_I.h"

const INT8U cAddr0xAA[6]={
    0xAA,0xAA,0xAA,0xAA,0xAA,0xAA
};													// 0xAA 地址
const INT8U cAddr0x99[6]={
    0x99,0x99,0x99,0x99,0x99,0x99
};													// 0x99 地址
const INT8U cAddr0xFF[6]={
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};													// 0xFF 地址
const INT8U cAddr0x00[6]={
    0x00,0x00,0x00,0x00,0x00,0x00
};													// 0x00 地址
const INT8U cAddr0xEE[6]={
    0xEE,0xEE,0xEE,0xEE,0xEE,0xEE
};													// 0xEE 地址

gCAC			        mCAC;								      // CAC全局参数
gDAU			        *mDAU;								// DAU队列全局参数指针
gMeter			        *mMeter;							// 电表队列结构全局参数指针
gNeighbor               *mNeighbor;                         // 邻居DAU队列全局参数指针
gYL				        mYL[6];								// 游离队列结构全局参数
gDLRf			        mRf;								    // 射频数据解析变量
gParse                  mParse;                                 // CAC全局功能参数开关

INT16U			mSaveCH[SUM_CH_NUM];				// 保存周边信道表
INT16U			mSaveCHALL[SUM_CH_NUM];				// 保存周边信道表
INT8U			mUpPath[MAX_LAYER*12];				// 上下路径
INT16U          cac_GetBroadDelayTime(INT16U tShiXi);


extern     INT8U   AT_IPR_info; // 波特率数值
//===============================================================================================
//  函数名称:        cac_Init
//  函数描述:   	    CAC软件初始化
//  入口参数:	 
//  返回值  :        
//  说明    ：  
//  修改记录:   < 作者 >      <修改日期>     < 版本 >      <描述>             
//               陈 炎       2013-10-30       1.0         添加网络规模标志bEnableCapacity初始化
//                                                        添加从节点主动上报开关bReport初始化
//                                                        添加事件上报开关 
//               陈 炎       2013-10-30       1.1         各种状态初始化封装成函数cac_IntoIdleState                                   
//===============================================================================================
void cac_Init()
{ 
    mDAU		= (gDAU*)drvBuf.pDAU;
    mMeter		= (gMeter*)drvBuf.pMeter;
    mNeighbor   = (gNeighbor*)drvBuf.pNeighbor;

    // 清空内存数据
    Co_SetArry_stat((INT8U*)(&mRf),0,sizeof(gDLRf));
    Co_SetArry_stat((INT8U*)(&mYL),0,sizeof(gYL)*6);

    mRf.aBuf	= drvBuf.pmRf;
    mRf.BufTx	= drvUartRf.aRfTxBuf;

    // 读取 CAC 的全局参数
    //  若异常
    if(drv_ReadData((INT8U*)(&mCAC),sizeof(gCAC),DRV_SAVE_READ_TYPE_CAC)==FALSE0)
    {
        //  清空mCAC结构体数据 
        Co_SetArry_stat((INT8U*)(&mCAC),0,sizeof(gCAC));

        //  矫正频道、场强门限
        mCAC.i1BigCH  = DEFAULT_CH;
        mCAC.i1Valve  = DEFAULT_RISS;
    }
    else if(mCAC.i4AutoInit != AUTO_INIT_SWITCH)
    {
        mCAC.i4AutoInit = AUTO_INIT_SWITCH;

        //  矫正频道、场强门限
        mCAC.i1Valve  = DEFAULT_RISS;
        cac_ClearAll();
        cac_SaveAll();
    }
    else
    {
        // 纠正频道
        if((mCAC.i1BigCH >= SUM_CH_NUM)
            ||(mCAC.i1BigCH == TEST_CH))
            mCAC.i1BigCH=DEFAULT_CH;

        // 纠正门限
        if(mCAC.i1Valve < MIN_RISS || mCAC.i1Valve > MAX_RISS)
            mCAC.i1Valve = DEFAULT_RISS;

        // 纠正 DAU 总数
        if(mCAC.i2AllNum>=MAX_DAU)
            mCAC.i2AllNum=0;

        if(mCAC.i2Neighbor>=MAX_DAU)
            mCAC.i2Neighbor=0;

        //  纠正网络规模
        if(mCAC.i2Capacity < 2 || mCAC.i2Capacity >512)
            mCAC.i2Capacity = 256;       
    }
    
    meter_send = 0;
    meter_succeed = 0;

    // 读取 DAU 队列
    Co_SetArry_stat4G(drvBuf.pDAU,0,MAX_DAU*sizeof(gDAU));
    //drv_ReadData(drvBuf.pDAU,mCAC.i2AllNum*sizeof(gDAU),DRV_SAVE_READ_TYPE_DAU);
    drv_ReadDAU(drvBuf.pDAU,mCAC.i2AllNum*sizeof(gDAU));

    // 读取电表队列
    drv_ReadData(drvBuf.pMeter,mCAC.i2MeterNum*sizeof(gMeter),DRV_SAVE_READ_TYPE_METER);

    // 读路径队列
    Co_SetArry_stat4G(drvBuf.pPath,0,MAX_DAU*DRV_LEN_ONE_PATH_ARRY);
    //drv_ReadData(drvBuf.pPath,mCAC.i2AllNum*DRV_LEN_ONE_PATH_ARRY,DRV_SAVE_READ_TYPE_PATH);
    drv_ReadPath(drvBuf.pPath,mCAC.i2AllNum*DRV_LEN_ONE_PATH_ARRY);
        
    // 读取场强表
    Co_SetArry_stat4G(drvBuf.pRiss,0,(MAX_DAU+1)*LEN_ONE_RISS);
    drv_ReadRiss(drvBuf.pRiss,(mCAC.i2AllNum+1)*LEN_ONE_RISS);

    // 读取邻居队列
    Co_SetArry_stat(drvBuf.pNeighbor,0,MAX_DAU*sizeof(gNeighbor));
    drv_ReadData(drvBuf.pNeighbor,mCAC.i2Neighbor*sizeof(gNeighbor),DRV_SAVE_READ_TYPE_NEIGHBOR);

    // 读取参数队列
    cac_ReadParse();
    
    if(DRV_LEN_ONE_DAU_ARRY>sizeof(gDAU))
    {
        drv_Printf("\nDAU队列空间大于实际空间，浪费！");
    }
    else if(DRV_LEN_ONE_DAU_ARRY<sizeof(gDAU))
    {
        drv_Printf("\nDAU队列空间小于实际空间，内存溢出！");
    }

    if(DRV_LEN_ONE_METER_ARRY>sizeof(gMeter))
    {
        drv_Printf("\n电表队列空间大于实际空间，浪费！");
    }
    else if(DRV_LEN_ONE_METER_ARRY<sizeof(gMeter))
    {
        drv_Printf("\n电表队列空间小于实际空间，内存溢出！");
    }

    mCAC.i1ReadMeterMaxTime = TIME_MAX_METER;                           //  延时参数复位，防止未初始化的模块上报错误数据

    if(mCAC.i2ShiXi == 0)
    {
        mCAC.i2BroadMaxTime = (INT16U)(MAX_DAU*3/10);     
    }
    else
    {
        mCAC.i2BroadMaxTime = cac_GetBroadDelayTime(mCAC.i2ShiXi); 
    }

    

    // 运行状态初始化
    cac_IntoIdleState();

    // 统计DAU的各种信息
    cac_CountDAU();

    // 算法初始化
    mth_Init(3);	

    // 抄表初始化
    init_Meter();

    InitProbeAll();

    mCAC.i2ScanAll  = 0;
    mCAC.i2ScanGood = 0;
    mCAC.i2ScanErr  = 0;

    // 默认开关全部关闭
    mParse.mode_spread = FALSE0;
    mParse.mode_NewScan = FALSE0;
    mParse.mode_collect_more = FALSE0;
    mParse.mode_collect_lost = FALSE0;
    mParse.mode_boardcast = FALSE0;

    //  主动上报路由运行信息
    //CACReport03H_F10();  
}

// 保存CAC信息
void cac_SaveCAC()
{
    // 统计DAU的各种信息
    cac_CountDAU();

    drv_SaveData((INT8U*)(&mCAC),sizeof(gCAC),DRV_SAVE_READ_TYPE_CAC);
}


void cac_SaveParse()
{
    drv_SaveData((INT8U*)(&mParse),sizeof(gParse),DRV_SAVE_READ_TYPE_PARSE1);
    drv_SaveData((INT8U*)(&mParse),sizeof(gParse),DRV_SAVE_READ_TYPE_PARSE2);
    drv_SaveData((INT8U*)(&mParse),sizeof(gParse),DRV_SAVE_READ_TYPE_PARSE3);
}

// 保存DAU信息
void cac_SaveDAU()
{
    drv_SaveDAU(drvBuf.pDAU,mCAC.i2AllNum*sizeof(gDAU));
}

// 保存所有信息
void cac_SaveAll()
{
    // 纠正频道
    if((mCAC.i1BigCH >= SUM_CH_NUM)
        ||(mCAC.i1BigCH == TEST_CH))
        mCAC.i1BigCH = DEFAULT_CH;

    // 纠正门限
    if(mCAC.i1Valve < MIN_RISS || mCAC.i1Valve > MAX_RISS)
        mCAC.i1Valve = DEFAULT_RISS;

    //  纠正网络规模
    if(mCAC.i2Capacity < 2 || mCAC.i2Capacity >512)
        mCAC.i2Capacity = 256;

    // 纠正 DAU 总数
    if(mCAC.i2AllNum>=MAX_DAU)
        mCAC.i2AllNum=0;

    // 纠正 邻居 总数
    if(mCAC.i2Neighbor>=MAX_DAU)
        mCAC.i2Neighbor=0;

    // 保存 CAC 信息
    cac_SaveCAC();

    // 保存 DAU 队列
    //drv_SaveData(drvBuf.pDAU,mCAC.i2AllNum*sizeof(gDAU),DRV_SAVE_READ_TYPE_DAU);
    drv_SaveDAU(drvBuf.pDAU,mCAC.i2AllNum*sizeof(gDAU));

    drv_stm32WDT();
    
    // 保存电表队列
    drv_SaveData(drvBuf.pMeter,mCAC.i2MeterNum*sizeof(gMeter),DRV_SAVE_READ_TYPE_METER);
    drv_stm32WDT();

    // 保存路径队列
    //drv_SaveData(drvBuf.pPath,mCAC.i2AllNum*DRV_LEN_ONE_PATH_ARRY,DRV_SAVE_READ_TYPE_PATH);
    drv_SavePath(drvBuf.pPath,mCAC.i2AllNum*DRV_LEN_ONE_PATH_ARRY);
    
    drv_stm32WDT();
    
    // 保存场强表
    drv_SaveRiss(drvBuf.pRiss,(mCAC.i2AllNum+1)*LEN_ONE_RISS);
    
    drv_stm32WDT();
}

void cac_ReadParse()
{
    gParse tParse1, tParse2, tParse3;
    
    Co_SetArry_stat((INT8U*)&mParse, 0xFF, sizeof(gParse));
    drv_ReadData((INT8U*)&tParse1, sizeof(gParse), DRV_SAVE_READ_TYPE_PARSE1);
    drv_ReadData((INT8U*)&tParse2, sizeof(gParse), DRV_SAVE_READ_TYPE_PARSE2);
    drv_ReadData((INT8U*)&tParse3, sizeof(gParse), DRV_SAVE_READ_TYPE_PARSE3);
    
    if(Cb_CmpArry_stat((INT8U*)&tParse2, (INT8U*)&tParse1, sizeof(gParse)))
    {
        Co_CpyArry_stat((INT8U*)&tParse2, (INT8U*)&mParse, sizeof(gParse));
    }
    else
    {
        if(Cb_CmpArry_stat((INT8U*)&tParse3, (INT8U*)&tParse1, sizeof(gParse)))
            Co_CpyArry_stat((INT8U*)&tParse3, (INT8U*)&mParse, sizeof(gParse));
        
        else if(Cb_CmpArry_stat((INT8U*)&tParse3, (INT8U*)&tParse2, sizeof(gParse)))
            Co_CpyArry_stat((INT8U*)&tParse3, (INT8U*)&mParse, sizeof(gParse));
    }
    
    // 纠正参数
    if(mParse.mode_spread>7)
    {
        Co_SetArry_stat((INT8U*)&mParse, 0, sizeof(gParse));
        cac_SaveParse();
    }
}

// 清空所有信息
void cac_ClearAll()
{
    // 索引清0，DAU队列无效（连带场强表、路由表无效）
    mCAC.i2AllNum=0;

    // 索引清0，电表队列无效
    mCAC.i2MeterNum=0;
    
    mCAC.i2DownDAU=0;
    mCAC.i2GoodDAU=0;
    mCAC.i2InNetDAU=0;
    
    mCAC.i2InNetDAU1=0;
    mCAC.i2InNetDAU2=0;
    mCAC.i2InNetDAU3=0;
    mCAC.i2MeterGood=0;
    mCAC.i2MeterAll=0;
    mCAC.i2MeterErr=0;
    mCAC.i2Instable=0;
    mCAC.i2Succeed1=0;
    mCAC.i2Succeed2=0;
    mCAC.i2Succeed3=0;
    
    // 串口使用的CAC参数恢复默认
    mCAC.i1RFPower          = 0;
    mCAC.i1ReadMeterMaxTime = TIME_MAX_METER;
}

// 统计 DAU 节点的数量
void cac_CountDAU()
{
    INT16U i;

    mCAC.i2UseDAU=0;
    mCAC.i2InNetDAU = 0;
    mCAC.i2DownDAU=0;	
    mCAC.i2ElseDAU=0;
    mCAC.i2GoodDAU=0;
    mCAC.i2BadDAU=0;

    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(dauF_Use(i)==TRUE1)
        {
            // 使用数量加1
            mCAC.i2UseDAU++;

            //  下载DAU
            if(mDAU[i].bDWrite==TRUE1 && mDAU[i].bDFind==FALSE0)
            {
                // 下载数量加1
                mCAC.i2DownDAU++;

                // 累计在网DAU
                if(mDAU[i].b2DNetAll>0)
                {
                    mCAC.i2InNetDAU++;
                }

                // 在网节点
                if(mDAU[i].b2DNetAll>0 && mDAU[i].bDGood==TRUE1)
                    mCAC.i2GoodDAU++;
                // 不在网节点
                else
                    mCAC.i2BadDAU++;
            }

            // 旁系 DAU
            if(mDAU[i].bDFind==TRUE1)
                mCAC.i2ElseDAU++;
        }
    }
}
// 查找节点 pAddr 在路径中的层次
INT8U cac_LayerInPath(INT8U *pAddr,INT8U *pPath)
{
    INT8U i;

    for(i=0;i<MAX_LAYER;i++)
    {
        if(Cb_CmpArry_stat(pPath+i*6,pAddr,6)==TRUE1)
            return (i+1);

        if(Cb_CmpArry_stat(pPath+i*6,cAddr0xAA,6)==TRUE1)
            return (i+1);

        if(Cb_CmpArry_stat(pPath+i*6,cAddr0x00,6)==TRUE1)
            return (i+1);

        if(Cb_CmpArry_stat(pPath+i*6,cAddr0xFF,6)==TRUE1)
            return (i+1);

        if(Cb_CmpArry_stat(pPath+i*6,cAddr0x99,6)==TRUE1)
            return (i+1);

        if(Cb_CmpArry_stat(pPath+i*6,cAddr0xEE,6)==TRUE1)
            return (i+1);
    }
    if(i>=MAX_LAYER)
    {
        return (MAX_LAYER);
    }

    return 0;	
}

//===============================================================================================
//  函数名称:        cac_UserAddDAU
//  函数描述:   	    用户增加一个DAU
//  入口参数:	 
//
//             1.  pAddr    <类型>  INT8U*
//                          <说明>  pAddr[0] ~ pAddr[5]存放待添加DAU地址
//
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			    陈炎       2013-09-26      1.0         添加注释,定义协议类型字段
//===============================================================================================
void cac_UserAddDAU(INT8U *pAddr)
{
    INT16U i;
    INT16U tN;
    INT8U tF;

    // 首先查找存在的节点
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(dauF_Use(i)==TRUE1 && Cb_CmpArry_stat(mDAU[i].aDAddr,pAddr,6)==TRUE1)
        {
            mDAU[i].bDWrite =   TRUE1;
            mDAU[i].bDFind  =   FALSE0;

#ifdef	ONLY_UART_2013
            //  更新协议类型
            mDAU[i].aDProType[1]=pAddr[6];            
#endif
            return;
        }
    }

    tF=FALSE0;

    // 补空位
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(dauF_Use(i)==FALSE0)
        {
            tF=TRUE1;
            tN=i;

            break;
        }
    }

    // 加在末尾
    if(tF==FALSE0 && mCAC.i2AllNum<MAX_DAU-1)
    {
        tF=TRUE1;
        tN=mCAC.i2AllNum;
        mCAC.i2AllNum++;
    }

    // 替换非嫡系DAU
    if(tF==FALSE0)
    {
        for(i=0;i<mCAC.i2AllNum;i++)
        {
            if(dauF_Down(i)==FALSE0)
            {
                tF=TRUE1;
                tN=i;

                break;
            }
        }
    }

    // 找到合适的新位置
    if(tF==TRUE1)
    {
        // 清空 DAU 队列信息
        Co_SetArry_stat((INT8U*)(&mDAU[tN]),0,sizeof(gDAU));

        // 清空场强表信息
        mth_ClearRiss(tN);

        // 清空路径信息
        mth_AddOne(tN);

        mDAU[tN].bDUse      =   TRUE1;
        mDAU[tN].bDWrite    =   TRUE1;
        mDAU[tN].bDFind     =   FALSE0;

        mDAU[tN].aDAddr[0]=pAddr[0];
        mDAU[tN].aDAddr[1]=pAddr[1];
        mDAU[tN].aDAddr[2]=pAddr[2];
        mDAU[tN].aDAddr[3]=pAddr[3];
        mDAU[tN].aDAddr[4]=pAddr[4];
        mDAU[tN].aDAddr[5]=pAddr[5];

#ifdef	ONLY_UART_2013
        mDAU[tN].aDProType[1]=pAddr[6];
#endif

        cac_GetShortID(mDAU[tN].aDShortID, mDAU[tN].aDAddr);

        // 新增节点个数+1
        mCAC.b10NewDAU++;
    }
}

// 用户增加一个电表
INT16U cac_UserAddMeter(INT8U *pAddr)
{
    INT16U i;
    INT16U tN;
    INT8U tF;

    // 首先查找存在的节点
    for(i=0;i<mCAC.i2MeterNum;i++)
    {
        if(mMeter[i].bMUse==TRUE1 && Cb_CmpArry_stat(mMeter[i].aMAddr,pAddr,6)==TRUE1)
        {
            mMeter[i].bMWrite=TRUE1;

            return i;
        }
    }

    tF=FALSE0;
    // 补空位
    for(i=0;i<mCAC.i2MeterNum;i++)
    {
        if(mMeter[i].bMUse==FALSE0)
        {
            tF=TRUE1;
            tN=i;

            break;
        }
    }

    // 加在末尾
    if(tF==FALSE0 && mCAC.i2MeterNum<MAX_DAU-1)
    {
        tF=TRUE1;
        tN=mCAC.i2MeterNum;

        mMeter[tN].bMFind=TRUE1;
        mCAC.i2MeterNum++;
    }

    // 找到合适的新位置
    if(tF==TRUE1)
    {
        mMeter[tN].bMUse=TRUE1;
        mMeter[tN].bMWrite=TRUE1;
        mMeter[tN].bMGood=FALSE0;

        mMeter[tN].aMAddr[0]=pAddr[0];
        mMeter[tN].aMAddr[1]=pAddr[1];
        mMeter[tN].aMAddr[2]=pAddr[2];
        mMeter[tN].aMAddr[3]=pAddr[3];
        mMeter[tN].aMAddr[4]=pAddr[4];
        mMeter[tN].aMAddr[5]=pAddr[5];

    }
    return tN;
}

// 发现一个 DAU
INT16U cac_FindDAU(INT8U *pAddr)
{
    INT16U i;
    INT16U tN;
    INT8U tF;

    // 判断是否是CAC
    if(pAddr[0]==0xFF
        &&pAddr[1]==0xFF
        &&pAddr[2]==0xFF
        &&pAddr[3]==0xFF
        &&pAddr[4]==0xFF
        &&pAddr[5]==0xFF
        )
        return 0xFFFF;

    // 判断是否是CAC
    if(pAddr[0]==0xAA
        &&pAddr[1]==0xAA
        &&pAddr[2]==0xAA
        &&pAddr[3]==0xAA
        &&pAddr[4]==0xAA
        &&pAddr[5]==0xAA
        )
        return 0xFFFF;


    // 首先查找存在DAU
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(dauF_Use(i)==TRUE1 && Cb_CmpArry_stat(pAddr,mDAU[i].aDAddr,6)==TRUE1)
        {
            return i;
        }
    }

    tF=FALSE0;
    // 补空位
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(dauF_Use(i)==FALSE0)
        {
            tF=TRUE1;
            tN=i;

            break;
        }
    }

    // 加在末尾
    if(tF==FALSE0 && mCAC.i2AllNum<MAX_DAU-1)
    {
        tF=TRUE1;
        tN=mCAC.i2AllNum;

        mCAC.i2AllNum++;
    }

    // 找到合适的新位置
    if(tF==TRUE1)
    {
        // 清空 DAU 队列信息
        Co_SetArry_stat((INT8U*)(&mDAU[tN]),0,sizeof(gDAU));

        // 清空场强表信息
        mth_ClearRiss(tN);

        // 清空路径信息
        mth_AddOne(tN);

        mDAU[tN].bDUse=TRUE1;
        mDAU[tN].bDFind=TRUE1;
        mDAU[tN].bDWrite=TRUE1;

        mDAU[tN].aDAddr[0]=pAddr[0];
        mDAU[tN].aDAddr[1]=pAddr[1];
        mDAU[tN].aDAddr[2]=pAddr[2];
        mDAU[tN].aDAddr[3]=pAddr[3];
        mDAU[tN].aDAddr[4]=pAddr[4];
        mDAU[tN].aDAddr[5]=pAddr[5];

        // CRC16 校验
        my_crc16(mDAU[tN].aDAddr,6,mDAU[tN].aDShortID);

        return tN;
    }

    return MAX_DAU;
}

// 发现一个 邻居
INT16U cac_FindNeighbor(INT8U *pAddr)
{
    INT16U tN, i;
    INT8U tF;

    // 判断是否是CAC
    if(pAddr[0]==0xFF
        &&pAddr[1]==0xFF
        &&pAddr[2]==0xFF
        &&pAddr[3]==0xFF
        &&pAddr[4]==0xFF
        &&pAddr[5]==0xFF
        )
        return 0xFFFF;

    // 判断是否是CAC
    if(pAddr[0]==0xAA
        &&pAddr[1]==0xAA
        &&pAddr[2]==0xAA
        &&pAddr[3]==0xAA
        &&pAddr[4]==0xAA
        &&pAddr[5]==0xAA
        )
        return 0xFFFF;

    // 首先查找存在DAU
    for(i=0;i<mCAC.i2Neighbor;i++)
    {
        if(neighborF_Use(i)==TRUE1 && Cb_CmpArry_stat(pAddr,mNeighbor[i].aNAddr,6)==TRUE1)
        {
            return i;
        }
    }

    tF=FALSE0;
    // 加在末尾
    if(tF==FALSE0 && mCAC.i2Neighbor<MAX_DAU-1)
    {
        tF=TRUE1;
        tN=mCAC.i2Neighbor;

        mCAC.i2Neighbor++;
    }

    // 找到合适的新位置
    if(tF==TRUE1)
    {
        // 清空 DAU 队列信息
        Co_SetArry_stat((INT8U*)(&mNeighbor[tN]),0,sizeof(gNeighbor));

        mNeighbor[tN].bNUse=TRUE1;

        mNeighbor[tN].aNAddr[0]=pAddr[0];
        mNeighbor[tN].aNAddr[1]=pAddr[1];
        mNeighbor[tN].aNAddr[2]=pAddr[2];
        mNeighbor[tN].aNAddr[3]=pAddr[3];
        mNeighbor[tN].aNAddr[4]=pAddr[4];
        mNeighbor[tN].aNAddr[5]=pAddr[5];

        return tN;
    }

    return MAX_DAU;
}

// 查找一个DAU的序号
INT16U cac_CheckDAUNum(INT8U *pAddr)
{
    INT16U i;

    // 判断是否是CAC
    if(pAddr[0]==0xFF
        &&pAddr[1]==0xFF
        &&pAddr[2]==0xFF
        &&pAddr[3]==0xFF
        &&pAddr[4]==0xFF
        &&pAddr[5]==0xFF)
        return 0xFFFF;

    // 判断是否是CAC
    if(pAddr[0]==0xAA
        &&pAddr[1]==0xAA
        &&pAddr[2]==0xAA
        &&pAddr[3]==0xAA
        &&pAddr[4]==0xAA
        &&pAddr[5]==0xAA)
        return 0xFFFF;

    // 首先查找存在DAU
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(dauF_Use(i)==TRUE1 && Cb_CmpArry_stat(pAddr,mDAU[i].aDAddr,6)==TRUE1)
        {
            return i;
        }
    }

    return MAX_DAU;
}

// 查找一个DAU的通讯协议类型
INT8U cac_GetProType(INT8U *pAddr)
{
    INT16U i = cac_CheckDAUNum(pAddr);
    
    // 在档案内
    if(i < MAX_DAU)
    {
        return mDAU[i].aDProType[1];
    }
    else
    {
        return 0;
    }  
}
// 查找一个Neighbor的序号
INT16U cac_CheckNeighborNum(INT8U *pAddr)
{
    INT16U i;

    // 判断是否是CAC
    if(pAddr[0]==0xFF
        &&pAddr[1]==0xFF
        &&pAddr[2]==0xFF
        &&pAddr[3]==0xFF
        &&pAddr[4]==0xFF
        &&pAddr[5]==0xFF
        )
        return 0xFFFF;

    // 判断是否是CAC
    if(pAddr[0]==0xAA
        &&pAddr[1]==0xAA
        &&pAddr[2]==0xAA
        &&pAddr[3]==0xAA
        &&pAddr[4]==0xAA
        &&pAddr[5]==0xAA
        )
        return 0xFFFF;

    // 首先查找存在DAU
    for(i=0;i<mCAC.i2Neighbor;i++)
    {
        if(neighborF_Use(i)==TRUE1 && Cb_CmpArry_stat(pAddr,mNeighbor[i].aNAddr,6)==TRUE1)
        {
            return i;
        }
    }

    return MAX_DAU;
}

// 通信延时相关参数
INT8U YanShiCanShu(INT8U* xAddr)
{
    INT16U tNum=cac_CheckDAUNum(xAddr);

    switch (mDAU[tNum].b4DLayerSon)
    {
    case 1:
        return 1;

    case 2:
        return 2;

    case 3:
        return 3;

    case 4:
        return 5;

    case 5:
        return 6;

    case 6:
        return 7;
    }

    return 8;
}

// 查找一个短地址的序号
INT16U cac_CheckShortID(INT8U *pShort,INT8U *p6Byte)
{
    INT16U i;

    //短地址为00
    if(pShort[0]==0&&pShort[1]==0)
    {
        if(p6Byte!=(INT8U*)0)
        {
            p6Byte[0]=0xEE;
            p6Byte[1]=0xEE;
            p6Byte[2]=0xEE;
            p6Byte[3]=0xEE;
            p6Byte[4]=0xEE;
            p6Byte[5]=0xEE;
        }

        return MAX_DAU;
    }

    if(pShort[0]==0xAA&&pShort[1]==0xAA)
    {
        if(p6Byte!=(INT8U*)0)
        {
            p6Byte[0]=0xAA;
            p6Byte[1]=0xAA;
            p6Byte[2]=0xAA;
            p6Byte[3]=0xAA;
            p6Byte[4]=0xAA;
            p6Byte[5]=0xAA;
        }
        return NUM_CAC;
    }

    // 首先查找存在DAU
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(dauF_Son(i) == TRUE1
            && mDAU[i].aDShortID[0]==pShort[0] 
        && mDAU[i].aDShortID[1]==pShort[1] )
        {
            if(p6Byte!=(INT8U*)0)
            {
                p6Byte[0]=mDAU[i].aDAddr[0];
                p6Byte[1]=mDAU[i].aDAddr[1];
                p6Byte[2]=mDAU[i].aDAddr[2];
                p6Byte[3]=mDAU[i].aDAddr[3];
                p6Byte[4]=mDAU[i].aDAddr[4];
                p6Byte[5]=mDAU[i].aDAddr[5];
            }
            return i;
        }
    }

    if(p6Byte!=(INT8U*)0)
    {
        p6Byte[0]=0xEE;
        p6Byte[1]=0xEE;
        p6Byte[2]=0xEE;
        p6Byte[3]=0xEE;
        p6Byte[4]=0xEE;
        p6Byte[5]=0xEE;
    }

    return MAX_DAU;
}

// 获取一个节点的短地址
void    cac_GetShortID(INT8U *pSAddr, INT8U *pLAddr)
{
    INT16U i;
    INT8U  nShort[2] = {0x01, 0xB0};
    INT8U  fSameID = FALSE0;

    if (Cb_CmpArry_stat(pLAddr, mCAC.aCAddr,6)==TRUE1)
    {
        nShort[0] = 0xAA;
        nShort[1] = 0xAA;
    }

    for (i=0; i<mCAC.i2AllNum; i++)
    {
        // 仅处理档案内节点
        if(dauF_Son(i) == TRUE1)
        {
          // 查找有无重名
          if (mDAU[i].aDShortID[0]==pLAddr[0] && mDAU[i].aDShortID[1]==pLAddr[1])
          {
              fSameID = TRUE1;
          }

          // 记录最大重名
          if((mDAU[i].aDShortID[1] & 0xF0) == 0xB0)
          {
              nShort[0] = mDAU[i].aDShortID[0];
              nShort[1] = mDAU[i].aDShortID[1];
          }
        }
    }

    // 若重名
    if (TRUE1 == fSameID)
    {
        pSAddr[0] = nShort[0]+1;
        pSAddr[1] = (0xFF == nShort[0])? nShort[1]+1 : nShort[1];
    }
    else
    {
        pSAddr[0] = pLAddr[0];
        pSAddr[1] = pLAddr[1];
    }

}

// 查找一个长地址的短地址
INT8U cac_LongToShort(INT8U *pShort, INT8U *pLong)
{
    INT16U tDAUN;

    tDAUN = cac_CheckDAUNum(pLong);
    if (tDAUN<MAX_DAU && dauF_Son(tDAUN))
    {
        pShort[0] = mDAU[tDAUN].aDShortID[0];
        pShort[1] = mDAU[tDAUN].aDShortID[1];

        return TRUE1;
    }
    else
    {
        pShort[0] = pLong[0];
        pShort[1] = pLong[1];
        pShort[2] = pLong[2];
        pShort[3] = pLong[3];
        pShort[4] = pLong[4];
        pShort[5] = pLong[5];

        return FALSE0;
    }
}

// CAC 场强值转换
INT8U cac_CACRissToVal(INT8U pRiss)
{
//    if(pRiss>mCAC.i1Valve)
//        return 0;
//    if(pRiss<mCAC.i1Valve && pRiss>96)
//        return 1;
    
//    if(pRiss>96)
//        return 0;
//    if(pRiss>40)
//        return ((96-pRiss)>>2)+1;
//    if(pRiss>0)
//        return 15;
//    return 0;
    
    if (pRiss>102)
        return 1;
    if (pRiss>60)
        return ((102-pRiss)/3)+1;
    if (pRiss>0)
        return 15;
    return 0;
}

// 底噪值转换
INT8U cac_NoiseToVal(INT8U pNoise)
{
    if (pNoise>99)
        return 0;
    if (pNoise>43)
        return ((99-pNoise)/4)+1;
    if (pNoise>0)
        return 15;
    return 0;
}

// DAU 场强值转换
INT8U cac_DAURissToVal(INT8U pRiss)
{
    return cac_CACRissToVal(pRiss);
}

// 场强矫正,pRiss为处理前场强值，pNoise为节点pNum底噪
INT8U cac_CorrectRssi(INT8U pRiss, INT8U pNoise)
{
    if(pNoise>100)
        pNoise=0;
    else
        pNoise=100-pNoise;

    if(pRiss>pNoise)
        return (pRiss-pNoise);

    return 0;
}
// 2 字节路径转 6 字节路径
void cac_2ByteTo6Byte(INT16U *p2Byte,INT8U *p6Byte,INT8U pFlgUp)
{
    INT8U k;
    INT16U tN;

    for(k=0;k<MAX_LAYER*2-1;k++)
    {
        if(pFlgUp==FALSE0 && k>=MAX_LAYER-1)
            break;

        tN=p2Byte[k];

        if(tN>=MAX_DAU||dauF_Use(tN)==FALSE0)
        {
            p6Byte[k*6+0]=0xFF;
            p6Byte[k*6+1]=0xFF;
            p6Byte[k*6+2]=0xFF;
            p6Byte[k*6+3]=0xFF;
            p6Byte[k*6+4]=0xFF;
            p6Byte[k*6+5]=0xFF;
        }
        else
        {
            p6Byte[k*6+0]=mDAU[tN].aDAddr[0];
            p6Byte[k*6+1]=mDAU[tN].aDAddr[1];
            p6Byte[k*6+2]=mDAU[tN].aDAddr[2];
            p6Byte[k*6+3]=mDAU[tN].aDAddr[3];
            p6Byte[k*6+4]=mDAU[tN].aDAddr[4];
            p6Byte[k*6+5]=mDAU[tN].aDAddr[5];
        }
    }
}

// 6 字节路径转 2 字节路径
void cac_6ByteTo2Byte(INT16U *p2Byte,INT8U *p6Byte,INT8U pLayer)
{
    INT8U k;
    INT16U i;

    if(pLayer>MAX_LAYER)
        return;

    for(k=0;k<=pLayer;k++)
    {
        for(i=0;i<mCAC.i2AllNum;i++)
        {
            if(dauF_In(i)==TRUE1
                && mDAU[i].aDAddr[0]==p6Byte[k*6+0]
            && mDAU[i].aDAddr[1]==p6Byte[k*6+1]
            && mDAU[i].aDAddr[2]==p6Byte[k*6+2]
            && mDAU[i].aDAddr[3]==p6Byte[k*6+3]
            && mDAU[i].aDAddr[4]==p6Byte[k*6+4]
            && mDAU[i].aDAddr[5]==p6Byte[k*6+5]
            )
                break;
        }

        if(i>=mCAC.i2AllNum)
        {
            p2Byte[k]=0xEEEE;
        }
        else
        {
            p2Byte[k]=i;
        }
    }
}

// 读取射频接收长度
INT16U drv_ReadRxRFLen()
{
    return drvUartRf.i2RfRxLen;
}

INT8U cac_ChoseCH(INT8U *pRssi)
{
    INT8U i, j, tRssi;
    INT8U tCH=0, tBest=0xFF;
    
    for(i=0; i<4; i++)
    {
        j = i<<1;
        tRssi = (pRssi[j]<pRssi[j+1])?pRssi[j]:pRssi[j+1];
        
        if(tRssi<tBest)
        {
            tBest = tRssi;
            tCH = i;
        }
    }
    return tCH;
}

// 读取信道遍历结果
void cac_CheckPathCH(INT16U pNum, INT8U *pBuf, INT8U pNo)
{
    INT8U i, j, tLayer, No=2;
    INT8U tmp;
    INT8U tRssi[4];

    tLayer = mth_3PathLayer(pNum, pNo);
    for(i=0; i<tLayer; i++)
    {
        // 读取父节点场强
        for(j=0; j<4; j++)
        {
            tRssi[j] = mDAU[pNum].aScanRssi[pNo][j+8*i];
        }
        // 读取子亲节点场强
        for(j=0; j<4; j++)
        {
            // 选择短板
            if(tRssi[j]<mDAU[pNum].aScanRssi[pNo][4+j+8*i]
            ||mDAU[pNum].aScanRssi[pNo][4+j+8*i]==0)
                tRssi[j] = mDAU[pNum].aScanRssi[pNo][4+j+8*i];
        }
        tmp=0xFF;
#if     BEST_CHANNEL
        // 选择四个频道最好的
        for(j=0; j<4; j++)
        {
            if(tRssi[j]<=tmp && tRssi[j]!=0)
            {
                tmp = tRssi[j];
                No = j;
            }
        }
        // 全部为0, 使用0频道
        if(tmp==0xFF)
            No = 0;
        pBuf[i] = No;
#else
        // 先选择工作频道好的
        for(j=2; j<4; j++)
        {
            if(tRssi[j]<=tmp && tRssi[j]!=0)
            {
                tmp = tRssi[j];
                No = j;
            }
        }
        // 全部为0, 先看测量1
        if(tmp==0xFF)
        {
            if(tRssi[1]!=0)
                pBuf[i] = 1;
            else
                pBuf[i] = 0;
        }
        else
            pBuf[i] = No;
#endif
    }
}

// 读取信道遍历结果, 上下不同频
void cac_CheckPathCH2(INT16U pNum, INT8U *pBuf, INT8U pNo)
{
    INT8U i, j, tLayer, scanNo=pNo-3;
    INT8U tRssi, tmp, start;

#if     BEST_CHANNEL
    start = 0;
#else
    start = 2;
#endif

    tLayer = mth_3PathLayer(pNum, pNo);
    for(i=0; i<tLayer; i++)
    {
        tmp=0xFF;
        // 读取子节点场强, 及选择最优下行信道
        for(j=start; j<4; j++)
        {
            tRssi = mDAU[pNum].aScanRssi[scanNo][4+j+8*i];
            if(tRssi<=tmp && tRssi!=0)
            {
                tmp = tRssi;
                pBuf[i] = j;
            }
        }
        if(tmp==0xFF)
            pBuf[i] = 0;
        
        tmp=0xFF;
        // 读取父节点场强, 及选择最优上行信道
        for(j=start; j<4; j++)
        {
            tRssi = mDAU[pNum].aScanRssi[scanNo][j+8*i];
            if(tRssi<=tmp && tRssi!=0)
            {
                tmp = tRssi;
                pBuf[MAX_LAYER+i] = j;
            }
        }
        if(tmp==0xFF)
            pBuf[i] = 0;
    }
}

//===============================================================================================
//  函数名称:        cac_SetSystemTime
//  函数描述:   	    系统时钟设置，校准CAC时间
//  入口参数:	 
//
//             1.  pTime       <类型>  stTime*
//                                 <说明>  结构体指针，指向一个stTime型结构体
//                                                   该结构体保存了时钟信息
//  返回值  :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			    陈 炎      2013-09-26      1.0          生成函数
//===============================================================================================
void cac_SetSystemTime(stTime* pTime)
{
    gSysTime.count        =      pTime->count;
    gSysTime.hour         =       pTime->hour;    
}

//根据时隙号计算广播延时，每个时隙300ms即0.3s
INT16U cac_GetBroadDelayTime(INT16U tShiXi)
{
    INT8U i = 0;

    i = tShiXi %10;

    //  四舍五入取整
    switch(i)
    {
    case 2:
        return (tShiXi*3/10 +1);
    case 3:
        return (tShiXi*3/10 +1);
    case 5:
        return (tShiXi*3/10 +1);
    case 6:
        return (tShiXi*3/10 +1);
    case 9:
        return (tShiXi*3/10 +1);
    default:
        break;
    }
    return (tShiXi*3/10);
}

void cac_CalPath()
{
    INT16U i;
    
    mth_3PathInit(2);

	for(i=0;i<mCAC.i2AllNum;i++)
		mth_3PathSet(i);

	mth_3PathShortClearAll();

	mth_3PathShortPathAll();
}

// 设置主动上报标识
void cac_SetEventFlag()
{
    mCAC.bEvent = TRUE1;
}

// 清除主动上报标识
void cac_CleanEventFlag()
{
    mCAC.bEvent = FALSE0;
}

// 按底噪情况对频点进行排序,随机选择底噪小的一个频道
INT8U cac_AutoChannel(void)
{
    INT8U   avg1[SUM_CH_ALL] = {0};             // 频点平均底噪
    INT8U   avg[SUM_CH_NUM] = {0};              // 频组平均底噪
    INT8U   seq[SUM_CH_NUM];                    // 频点号
    INT16U  i;                                  // 节点索引
    INT8U   m;                                  // 频点索引
    INT32U  sum;                                // 某频点底噪和
    INT16U  cnt;                                // 某频点合法DAU数

    // 计算平均底噪
    for (m=0; m<SUM_CH_ALL; m++)
    {
        // 累加和、计数清零
        sum     =   0;
        cnt     =   0;

        // 计算底噪和
        for (i=0; i<mCAC.i2AllNum; i++)
        {
            // 若节点累计在网
            if (TRUE1 == dauF_In(i))
            {
                // 正常范围60~130
                if (mDAU[i].aDBGNoise[m] <=130  &&  mDAU[i].aDBGNoise[m] >= 60)
                {
                    sum     +=  mDAU[i].aDBGNoise[m];
                }
                else
                {
                    sum     +=  100; 
                }

                // 计数+1
                cnt++;
            } 
        }

        // 计算底噪平均值
        avg1[m]  =   (INT8U)(sum/cnt);
    }
    
    // 串口打印66频点信息
    //uart_ReportData(NULL, avg1, 66, 0xF0, 106);
    
    // 计算频组平均底噪
    for (m=0; m<SUM_CH_NUM; m++)
    {
        // 频道序号赋值
        seq[m] = m;

        // 频组底噪均值（两频点取小）
        if (avg1[2*m+2] <= avg1[2*m + 3])
        {
            avg[m] = avg1[2*m+2];
        }
        else
        {
            avg[m] = avg1[2*m + 3];
        }
        avg[m] = avg1[2*m+2];
    }

    // 排序频点
    Co_ArryDownSort(seq, avg, SUM_CH_NUM);

    // 查找前1/2频道
    for (m=0; m<SUM_CH_NUM/2; m++)
    {
        // 工作频道已再其中
        if ((1+seq[m]) == mCAC.i1BigCH)
        {
            return 0;
        }
    }

    // 随机选择前1/2频道
    m = (INT8U)Csv_SandGet_stat(SUM_CH_NUM/2);
    return seq[m]+1;
}

//  CAC状态标志位初始化
void cac_IntoIdleState()
{
    mCAC.bSetup     =   FALSE0;
    mCAC.bCall      =   FALSE0;
    mCAC.bSetCH     =   FALSE0;
    mCAC.bBroad     =   FALSE0;
    mCAC.bMeter     =   FALSE0;
    mCAC.bScan      =   FALSE0;
    mCAC.bMeterAll  =   FALSE0;
    mCAC.bOptimize  =   FALSE0;
    mCAC.bInitXB    =   FALSE0;
}

void cac_CountInDAU( INT8U xType )
{
    INT16U i, num=0;
    
    for(i=0; i<mCAC.i2AllNum; i++)
    {
        if(dauF_Good(i)==TRUE1 && mDAU[i].bDFind==FALSE0)
            num++;
    }
    
    if(xType == TRUE1)
        mCAC.i2InNetDAU1 = num;
    else if(xType == FALSE2)
        mCAC.i2InNetDAU2 = num - mCAC.i2InNetDAU1;
    else
        mCAC.i2InNetDAU3 = num - mCAC.i2InNetDAU1 - mCAC.i2InNetDAU2;
}

// 获取试探标记
INT8U ReadProbeFlg(INT16U xNum)
{
    return mDAU[xNum].ProbeFlg;
}

// 写试探标记
void WriteProbeFlg(INT16U xNum, INT8U xFlg)
{
    mDAU[xNum].ProbeFlg = (xFlg>0) ? TRUE1 : FALSE0;
}

// 获取试探序号
INT16U ReadProbeSeq(INT16U xNum)
{
    return mDAU[xNum].ProbeSeq;
}

// 写试探序号
void WriteProbeSeq(INT16U xNum, INT16U xSeq)
{
    mDAU[xNum].ProbeSeq = xSeq;
}

// 初始化试探参数
void InitProbeAll(void)
{
    INT16U i;

    for (i=0; i<mCAC.i2AllNum ;i++)
    {
        if (mDAU[i].bDUse == TRUE1)
        {
            WriteProbeSeq(i, 0);
        }
    }
}