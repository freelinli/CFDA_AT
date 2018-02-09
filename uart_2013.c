#include	"prio_II.h"



extern  INT8U	uartTxBuf[UART_ARG_1_LEN];			// 定义串口发送缓冲区
extern  INT8U	uart_path[36];
extern  INT8U   tempBuf22[256];
extern  INT8U   backMeter;
extern INT16U TskOptimizeLimitTime;
INT16U      xb_Capacity         ( INT16U pN );
INT8U		set_setup_son		( void );			// 启动嫡系组网
INT8U		set_setup_all		( void );			// 启动全网组网
void		set_call_all		( void );			// 启动全网点名
void		set_updata			( void );			// 启动网络升级
void		set_set_ch			( void );			// 启动自动频道
void		uart_Path2ByteTo6Byte(INT16U pNum, INT8U *pPath);									// 路径处理函数
INT8U		uart_DAUType		(INT16U pNum,INT8U pType);									// 节点类型处理
void		rf_BroadcastTime	(INT8U *pBuf,  INT8U pLen);									// 广播校时
void        set_optimize();                           //  网络优化任务
void        LongToPanID(INT8U *pByte6,INT8U *pByte2);

#ifdef		ONLY_UART_2013

INT8U gFlgUrtCmpFile    =   FALSE0;
INT8U FlagNew = FALSE0;                                                   //  主动注册开关默认关闭
INT8U FlagNewNotCollect = TRUE1;                                  //  HT主动注册暂不支持采集器模式
INT16U gDAUNum;

extern INT8U   flag_at_meter;

// 通信延时相关参数
INT8U YanShiCanShu(INT8U* xAddr);
INT16U cac_GetBroadDelayTime(INT16U t);
INT8U sub_adf7020_read_rssi(void);
void drv_stm32WDT();
INT8U collect_meter(INT16U pNum);
INT8U sub_m25pe16_read_page(INT16U xpage,INT8U *xbuf);
INT8U Get645FrameType(INT8U *p);
INT8U  sub_AX5043_set_test_mode(INT8U p);

//===============================================================================================
//  函数名称:        uart_reaportNew
//  函数描述:   	    上报新节点信息，用于从节点主动注册被激活时的上报新发现节点
//  入口参数:	 
//  返回值  :        
//  说明    ：  用06H F4格式上报
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>             
//                             陈炎    2013-10-12            1.0           生成函数
//                             陈炎    2013-10-30            1.0          添加采集器上报结束条件：遍历完队列
//                             陈炎    2013-11-05            1.0          两帧间发送延迟由0.5s改为2s
//===============================================================================================
void uart_reaportNew()
{
    INT8U tN;
    INT16U i,j;
    INT8U *pBuf=tempBuf22;
    INT8U ReportNum = 0;                                                                              //  采集器已上报电表数

    drv_Printf("\n		搜表结果上报 ");
    // 上报发现电表信息
    tN=0;
    for(i=0;i < mCAC.i2AllNum;i++)
    {
        if(mDAU[i].bDDevType == 1)                                                                 //  若是电表
        {
            //  构造发送数据
            pBuf[1+tN*9] = mDAU[i].aDAddr[0];                                         //  从节点地址
            pBuf[2+tN*9] = mDAU[i].aDAddr[1];
            pBuf[3+tN*9] = mDAU[i].aDAddr[2];
            pBuf[4+tN*9] = mDAU[i].aDAddr[3];
            pBuf[5+tN*9] = mDAU[i].aDAddr[4];
            pBuf[6+tN*9] = mDAU[i].aDAddr[5];
            pBuf[7+tN*9] = mDAU[i].aDProType[1];                                    //  通信协议类型
            pBuf[8+tN*9] = (INT8U)i;                                                        //  从节点序号
            pBuf[9+tN*9] = (INT8U)(i >> 8);                                              //
            tN++;
        }

        if((tN >= 20) || (i == (mCAC.i2AllNum - 1)))                                  //  达到20个或者遍历完所有节点
        {
            pBuf[0] = tN;                                                                                   //  从节点数量
            uart_ReportData((INT8U*)0,pBuf,tN*9+1,6,1);
            drv_stm32WDT();
            drv_Delay10ms(200);
            tN = 0;           
        }
    }

    //  上报发现采集器信息
    tN = 0;
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if((mDAU[i].bDDevType == 0) && (mDAU[i].b6DMeterSum != 60))
        {
            //  构造节点信息
            pBuf[0]  = 1;                                                                      //  1个采集器报1帧
            pBuf[1]  = mDAU[i].aDAddr[0];                                         //  从节点地址
            pBuf[2]  = mDAU[i].aDAddr[1];
            pBuf[3]  = mDAU[i].aDAddr[2];
            pBuf[4]  = mDAU[i].aDAddr[3];
            pBuf[5]  = mDAU[i].aDAddr[4];
            pBuf[6]  = mDAU[i].aDAddr[5];
            pBuf[7]  = 0xFF;                                                            //  通信协议类型为FF，表明是采集器
            pBuf[8]  = (INT8U)i;                                                        //  从节点序号
            pBuf[9]  = (INT8U)(i >> 8);                                              //
            pBuf[10] = 0;                                                                    //  设备类型为0：采集器
            pBuf[11] = (INT8U)mDAU[i].b6DMeterSum;                  //  附属节点总数量M

            //  构造节点从属节点信息
            for(j = 0;j < mCAC.i2MeterNum;j++)                                   
            {
                //  若该电表归该DAU管
                if((mMeter[j].aMDAU[0] == mDAU[i].aDAddr[0])
                    &&(mMeter[j].aMDAU[1] == mDAU[i].aDAddr[1])
                    &&(mMeter[j].aMDAU[2] == mDAU[i].aDAddr[2])
                    &&(mMeter[j].aMDAU[3] == mDAU[i].aDAddr[3])
                    &&(mMeter[j].aMDAU[4] == mDAU[i].aDAddr[4])
                    &&(mMeter[j].aMDAU[5] == mDAU[i].aDAddr[5]))          
                {
                    pBuf[13+tN*7] = mMeter[j].aMAddr[0];                   //  电表tN地址
                    pBuf[14+tN*7] = mMeter[j].aMAddr[1];
                    pBuf[15+tN*7] = mMeter[j].aMAddr[2];
                    pBuf[16+tN*7] = mMeter[j].aMAddr[3];
                    pBuf[17+tN*7] = mMeter[j].aMAddr[4];
                    pBuf[18+tN*7] = mMeter[j].aMAddr[5];
                    pBuf[19+tN*7] = mMeter[j].i1MMeterType;                   //  通信协议类型

                    tN++;
                }

                if((tN >= 8) || (j == (mCAC.i2MeterNum - 1)))
                {
                    pBuf[12]    =     tN;                                                           //  本次上报总数量m
                    ReportNum += tN;                                                          //  更新累计上报数量
                    uart_ReportData((INT8U*)0,pBuf,tN*7+13,6,4);     
                    drv_stm32WDT();
                    drv_Delay10ms(200);
                    tN = 0;

                    //  若该采集器上报结束
                    if((ReportNum == (INT8U)mDAU[i].b6DMeterSum) 
                        ||(j == (mCAC.i2MeterNum - 1)))                            //  若已上报完或已遍历完电表队列
                    {
                        ReportNum = 0;
                        break;
                    }
                }
            }
        }
    }

    // 再等待500ms
    drv_Delay10ms(200);

    // 发送结束信息
    pBuf[0]=2;
    uart_ReportData((INT8U*)0,pBuf,1,6,3);

#if TEST_COLLECT_METER 
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if((mDAU[i].bDDevType == 0) && (mDAU[i].b6DMeterSum == 60))
        {
            drv_Printf("\n %d 老版本DAU[%d] =",mDAU[i].b6DMeterSum,i);
            drv_PrintfDAU(mDAU[i].aDAddr);
        }
    }
#endif

    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(mDAU[i].bDFind == TRUE1)
        {
            mDAU[i].bDUse = FALSE0;
        }
    }
    FlagNew = FALSE0;                                                                                                //  搜表结束


#if TEST_COLLECT_METER 
    drv_Printf("\n\n删除旁系DAU后");
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(mDAU[i].bDUse == TRUE1)
        {
            drv_Printf("\n DAU[%2d] [bDWrite = %d  bDFind = %d]=",i,mDAU[i].bDWrite, mDAU[i].bDFind);
            drv_PrintfDAU(mDAU[i].aDAddr);
            drv_Printf("\tType = %d  Layer = %d  b4DLayerALL = %d",mDAU[i].bDDevType,mDAU[i].b4DLayerSon,mDAU[i].b4DLayerALL);
        }
    }
#endif
}

//===============================================================================================
//  函数名称:        Data03HF10
//  函数描述:   	    本地通信模块运行模式数据写入
//  入口参数:	 
//
//             1.  xBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，数据写入区域的首地址，
//                                                   常指向用户数据区的数据单元
//  返回值  :        返回写入字节数
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-26           1.0           修改函数和注释
//                             陈炎    2013-10-10           1.0          修改版本信息不是LSB输出问题
//===============================================================================================
INT8U Data03HF10(INT8U *xBuf)
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


//===============================================================================================
//  函数名称:        CACReport03H_F10
//  函数描述:   	    主动上报本地模块运行模式
//  入口参数:	 
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-29           1.0           添加注释
//===============================================================================================
void CACReport03H_F10()
{
    INT8U length = 0;
    length = Data03HF10(uartTxBuf);                                                      //  写入运行模式信息                                                                                       
    uart_ReportData((INT8U*)0,uartTxBuf,length,3,10);                       //  上报数据
}

//===============================================================================================
//  函数名称:        CACReport03H_F1
//  函数描述:   	    主动上报厂家信息
//  入口参数:	 
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-29           1.0           添加注释
//===============================================================================================
void CACReport03H_F1()
{
    uartTxBuf[0]='C';                                                                                   //  写入运行模式信息 
    uartTxBuf[1]='F';                                                   
    uartTxBuf[2]='M';
    uartTxBuf[3]='J';
    uartTxBuf[4]=DT_DATA;
    uartTxBuf[5]=DT_MOTH;
    uartTxBuf[6]=DT_YEAR;
    uartTxBuf[7]=DT_EDITIOM_L;
    uartTxBuf[8]=DT_EDITIOM_H;
    uart_ReportData((INT8U*)0,uartTxBuf,9,3,1);                                 // 上报数据
}

//===============================================================================================
//  函数名称:        DL2013_AFN01_Fn1
//  函数描述:   	    硬件初始化，CAC复位
//  入口参数:	 
//
//  返回值  :        
//  说明    :       上/下行报文无数据单元
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN01_Fn1(void)    
{
    uart_Answer_Yes();                                                                           //  返回确认
    drv_Delay10ms(20);                                                                         //  等待发送完毕
    cac_SaveAll();
    drv_Resetcac();                                                                                  //  CAC复位
}

//===============================================================================================
//  函数名称:        DL2013_AFN01_Fn2
//  函数描述:   	 参数初始化，清除所有DAU档案
//  入口参数:	 
//
//  返回值  :        
//  说明    :        上/下行报文无数据单元
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN01_Fn2(void)    
{
    cac_ClearAll();
    cac_SaveAll();
    uart_Answer_Yes();                                                          //  返回确认
    drv_Delay10ms(20);                                                          //  等待发送完毕
}

//===============================================================================================
//  函数名称:        DL2013_AFN01_Fn3
//  函数描述:   	    数据区初始化，清除所有DAU通信信息，如场强表、路径
//  入口参数:	 
//
//  返回值  :        
//  说明    :       上/下行报文无数据单元
//  修改记录:       <作者>      <修改日期>     <版本 >      <描述>       
//			        陈炎    2013-09-25         1.0          生成函数
//===============================================================================================
void DL2013_AFN01_Fn3(void)    
{
    cac_ClearAll();                                                             //  清所有信息
    uart_Answer_Yes();                                                          //  返回确认
    cac_SaveAll();                                                              //  同步数据
    drv_Delay10ms(20);                                                          //  等待发送完毕
}

//===============================================================================================
//  函数名称:        DL2013_AFN02_Fn1
//  函数描述:   	 转发通信协议数据帧
//  入口参数:	 
//
//  返回值  :        
//  说明    :        与13H F1无通信延时相同
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           添加注释
//===============================================================================================
void DL2013_AFN02_Fn1(INT8U *pDAU,INT8U *pBuf,INT16U pLen)    
{
    INT16U tmpDAUNum = 0;
    INT8U tmpFrameType = 0;

    backMeter=0;

    //  数据长度异常
    if( (pLen > LEN_USER)                                                       
        || (pBuf[1]+2>pLen))                                                                                                          
    {
        drv_Printf("\n数据长度错误");
        uart_Answer_Err();
        return;
    }

    //  已有抄表任务
    if(mCAC.bMeter==TRUE1)                                                            
    {
        drv_Printf("\n已有抄表任务");
        return ;                                                                //  返回主节点忙
    }

    //  查找DAU序号
    tmpDAUNum = cac_CheckDAUNum(pDAU);
    drv_Printf("\n目标DAU[%3d] =  ",tmpDAUNum);
    drv_PrintfDAU(pDAU);

    //  节点不在档案
    if(tmpDAUNum >= MAX_DAU)                                                                                                    
    {
        drv_Printf("\n节点不在档案");
        uart_Answer_Unknown();                                                                            //  返回不在网
        return ;
    }

    //  若为透传命令
    if(pBuf[0] == 0x00)
    {
        //  更新DAU协议类型
        mDAU[tmpDAUNum].aDProType[0]= Get645FrameType(&(pBuf[2]));
    }
    //  若有协议字段
    else
    {
        //  解析报文帧类型
        tmpFrameType = Get645FrameType(&(pBuf[2]));

        mDAU[tmpDAUNum].aDProType[0] = tmpFrameType;
    }
    drv_Printf("\n==============================启动抄表=============================");
    set_Meter(tmpDAUNum,&(pBuf[2]),pBuf[1],mDAU[tmpDAUNum].aDProType[0],0,RF_ASK_METER_DATA);
}

//  查询开箱报警事件（扩展命令）02 02 00
void DL2013_AFN02_Fn2(INT8U *pDAU,INT8U *pBuf,INT16U pLen)
{
	INT16U i2Num;

    backMeter = 2;

    //  数据单元错误
    if((pLen < 2) || (pLen >= 2+ LEN_USER))
    {
        drv_Printf("\n数据长度溢出");
		uart_Answer_Err();    
        return;
    }

    //  已有抄表任务
	if(mCAC.bMeter==TRUE1)                                                            
    {
        drv_Printf("\n已有抄表任务 请稍后再查询开箱事件");
		return ;                                                                //  返回主节点忙
    }

    //  查找DAU序号
    i2Num = cac_CheckDAUNum(pDAU);   

    //  节点不在档案
    if(i2Num>= MAX_DAU)                                                                                                    
    {
        drv_Printf("\n节点不在档案");
        uart_Answer_Unknown();                                                                            //  返回不在网
        return ;
    }


    //  设置抄表
    set_Meter(i2Num,&(pBuf[2]),pLen-2,pBuf[0],0,RF_ASK_METER_DATA);
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn1
//  函数描述:   	    查询厂商代码和版本信息
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//             2.addrFlg     <类型>  INT8U
//                                  <说明>   回复帧地址域标志位，1表示有，0表示无         
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN03_Fn1(INT8U *pBuf, INT8U addrFlg)
{
    if(addrFlg==0)                                                                                    //  若查询CAC厂商代码和版本信息
    {
        pBuf[0]='C';                                                                                    //  写CAC厂商代码和版本信息到数据单元
        pBuf[1]='F';
        pBuf[2]='M';
        pBuf[3]='J';
        pBuf[4]=DT_DATA;                                                                        //  日
        pBuf[5]=DT_MOTH;                                                                      //  月
        pBuf[6]=DT_YEAR;                                                                         //  年
        pBuf[7]=DT_EDITIOM_L;                                                                //  版本号低字节
        pBuf[8]=DT_EDITIOM_H;	                                                                //  版本号高字节
        uart_Answer_Data(9);                                                                  //  返回数据
    }
    else                                                                                                      //  若查询DAU厂商代码和版本信息
    {
        pBuf[0]='C';                                                                                    //  写DAU厂商代码和版本信息到数据单元
        pBuf[1]='F';
        pBuf[2]='M';
        pBuf[3]='J';
        pBuf[4]=DT_DATA;
        pBuf[5]=DT_MOTH;
        pBuf[6]=DT_YEAR;
        pBuf[7]=0x02;
        pBuf[8]=00;
        uart_Answer_Data(9);                                                                  //  返回数据
    }
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn2
//  函数描述:   	    查询噪音值
//  入口参数:	 
//
//  返回值  :        
//  说明    :        不支持
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN03_Fn2(void) 
{
    uart_Answer_Nonsup();
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn3
//  函数描述:   	    查询从节点侦听信息
//  入口参数:	 
//
//  返回值  :        
//  说明    :        不支持
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN03_Fn3(void) 
{
    uart_Answer_Nonsup();
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn4
//  函数描述:   	    查询主节点地址
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :       
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN03_Fn4(INT8U *pBuf) 
{
    pBuf[0]=mCAC.aCAddr[0];                                                                  //  写CAC地址到数据单元                                                                                     
    pBuf[1]=mCAC.aCAddr[1];
    pBuf[2]=mCAC.aCAddr[2];
    pBuf[3]=mCAC.aCAddr[3];
    pBuf[4]=mCAC.aCAddr[4];
    pBuf[5]=mCAC.aCAddr[5];	
    uart_Answer_Data(6);                                                                      //  返回数据
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn5
//  函数描述:   	    查询主节点状态字和通信速率
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :       
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//                             陈炎    2013-10-11            1.0          修改通信速率数量误用信道数量
//===============================================================================================
void DL2013_AFN03_Fn5(INT8U *pBuf)    
{
    INT8U i;

    //  主节点状态字
    // 周期抄表模式=01b                          01   仅集中器主导(无线); 10   仅路由主导(载波) ; 11 两种都支持(双模异构)  
    //  主节点信道特征 =00b                 00 微功率无线；01 单相供电单相传输；10 单供三传；11三供三传 
    //  通信速率数量 =  1（0001b）   
    //  pBuf[1] 高四位备用，低四位信道数量
    pBuf[0]=0x41;	
    pBuf[1]=0x1f;                                                                                        //  信道实际数量32

    //  写通信速率
    for(i=0;i<(pBuf[0] & 0x0f);i++)
    {
        pBuf[i*2+2]=0;                                                                                  // D14~D0 通信速率，0表默认通信速率
        pBuf[i*2+3]=0x80;                                                                           //  D15: 速率单位标示; 1表示Kbps，0表示bps
    }
    // 返回数据
    uart_Answer_Data(2+2*(pBuf[0] & 0x0f));
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn6
//  函数描述:   	    查询主节点干扰情况
//  入口参数:	 
//
//  返回值  :        
//  说明    :        不支持
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN03_Fn6(void) 
{
    uart_Answer_Nonsup();
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn7
//  函数描述:   	    读取从节点监控最大超时时间
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                            <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			   陈炎       2013-09-25      1.0         生成函数
//===============================================================================================
void DL2013_AFN03_Fn7(INT8U *pBuf) 
{
    pBuf[0]=mCAC.i1ReadMeterMaxTime;	                                             //  提取从节点监控最大超时
    uart_Answer_Data(1);                                                                           //  返回数据
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn8
//  函数描述:   	    查询无线通信参数
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                            <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			   陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN03_Fn8(INT8U *pBuf)   
{
    pBuf[0]=mCAC.i1BigCHXX;		                                    //  无线信道组     0~63
    pBuf[1]=mCAC.i1RFPower;	                                        //  主节点发射功率 0最高；1次高；2次低；3最低	
    uart_Answer_Data(2);                                            //  返回数据
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn9
//  函数描述:   	    查询通信延时相关的广播时长
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :      
//             1.下行报文  pBuf[0] 存通信协议类型
//                                            pBuf[1] 存645帧报文长度L
//                                            pBuf[2] ~ pBuf[L+1] 存645帧内容
//
//             2.上行报文  pBuf[0] ~ pBuf[1] 存广播延时
//                                            pBuf[2] ~ pBuf[L+3] 存下行报文
//
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           添加注释
//                             陈炎    2013-10-11            1.0         调整长度，修改645帧最后2B丢失bug
//===============================================================================================
void DL2013_AFN03_Fn9(INT8U *pBuf)   
{
    INT8U i;
    INT16U j = cac_GetBroadDelayTime(mCAC.i2AllNum+5);//cac_GetBroadDelayTime(mCAC.i2ShiXi);//
    
    uartTxBuf[0]= (INT8U)j;                                                                           //  广播延时根据时隙计算
    uartTxBuf[1]= (INT8U)(j >> 8);                                                                //

    uartTxBuf[2]=pBuf[0];
    uartTxBuf[3]=pBuf[1];                                                                             //  提取

    for(i=0;i<pBuf[1];i++)                                                                               //  拷贝645帧
    {
        uartTxBuf[4+i]=pBuf[2+i];
    }

    for(i=0;i<uartTxBuf[3]+4;i++)                                                                 //  构造返回帧数据单元                                        
    {
        pBuf[i]=uartTxBuf[i];
    }

    uart_Answer_Data(uartTxBuf[3]+4);
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn10
//  函数描述:   	    查询本地通信模块运行模式
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           修改注释
//===============================================================================================
void DL2013_AFN03_Fn10(INT8U *pBuf)    
{
    INT8U length = 0;
    length = Data03HF10(pBuf);	                                                              //  写用户数据单元
    uart_Answer_Data(length);                                                                   //  返回数据
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn11
//  函数描述:   	    查询本地通信模块AFN索引
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           修改注释
//===============================================================================================
void DL2013_AFN03_Fn11(INT8U *pBuf)    
{
    INT8U i;

    for(i=0;i<32;i++)                                                                                      //  写入p[1]~p[32]初始化值
    {
        pBuf[1+i]=0;
    }

    switch(pBuf[0])                                                                                       //  根据查询，确定修改位
    {
    case 0x00:
        pBuf[1]    =    0x03;                                                                             //  F1~F2:  全支持
        break;

    case 0x01:
        pBuf[1]    =    0x07;                                                                             //  F1~F3:  全支持
        break;

    case 0x02:
        pBuf[1]    =    0x01;                                                                             //  F1:支持
        break;

    case 0x03:
        pBuf[1]    =    0xD9;                                                                           //  F1~F8:  F2，F3，F6不支持
        pBuf[2]    =    0x07;                                                                           //  F9~F11:  全支持
        pBuf[13]  =    0x08;                                                                           //  新增F100，100 = 8*12 + 4，对应p[13]的b3
        break;

    case 0x04:
        pBuf[1]    =    0x03;                                                                           //  F1~F3:  F3不支持
        break;

    case 0x05:
        pBuf[1]    =    0x1F;                                                                           //  F1~F5:  全支持
        pBuf[13]  =    0x18;                                                                          //  新增F100、F101
        break;

    case 0x06:
        pBuf[1]    =    0x1D;                                                                           //  F1~F5:  F2不支持                                                                     
        break;

    case 0x10:
        pBuf[1]    =    0x3B;                                                                          //  F1~F6:  F3不支持
        pBuf[13]  =    0x18;                                                                          //  新增F100、F101
        break;

    case 0x11:
        pBuf[1]    =    0x33;                                                                          //  F1~F6:  F3、F4不支持
        pBuf[13]  =    0x38;                                                                          //  新增F100、F101、F102
        break;

    case 0x12:
        pBuf[1]    =    0x07;                                                                           //  F1~F3:  全支持
        break;

    case 0x13:
        pBuf[1]    =    0x01;                                                                           //  F1:支持
        break;

    case 0x14:
        pBuf[1]    =    0x06;                                                                          //  F1~F3:  F1不支持
        break;

    case 0x15:
        pBuf[1]    =    0x01;                                                                           //  F1:支持
        break;
    }	

    uart_Answer_Data(33);                                                                       // 返回数据
}

//===============================================================================================
//  函数名称:        DL2013_AFN03_Fn100
//  函数描述:   	    查询场强门限
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0            生成函数
//===============================================================================================
void DL2013_AFN03_Fn100(INT8U *pBuf)
{
    pBuf[0] = mCAC.i1Valve;                                                                      //  场强门限，取值50~120
    uart_Answer_Data(1);                                                                          //  返回数据
}

//===============================================================================================
//  函数名称:        DL2013_AFN04_Fn1
//  函数描述:   	    链路接口发送测试
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           修改注释
//                                           陈炎    2013-10-28           1.0           修改drv_SendTest()
//===============================================================================================
void DL2013_AFN04_Fn1(INT8U *pBuf)
{
    drv_SendTest(pBuf[0], mCAC.i1BigCH);
    uart_Answer_Yes();
}

//===============================================================================================
//  函数名称:        DL2013_AFN04_Fn2
//  函数描述:   	    链路接口从节点点名
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           修改注释
//===============================================================================================
void DL2013_AFN04_Fn2(INT8U *pDAU,INT8U *pBuf)    
{
    set_call_all();
    uart_Answer_Yes();
    tsk_call_all();
}

//===============================================================================================
//  函数名称:        DL2013_AFN04_Fn3
//  函数描述:   	    本地通讯模块报文通信测试
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :       
//              1.    数据单元格式：pBuf[0]存测试通信速率，取值0~31；
//                                                            pBuf[1]~pBuf[6] 存目的地址
//                                                            pBuf[7]存通信协议类型，0表透明，1表97版，2表07版，其余保留
//                                                            pBuf[8]存报文长度
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           修改注释
//===============================================================================================
void DL2013_AFN04_Fn3(INT8U *pBuf)    
{
    uart_Answer_Nonsup();                               
}

void DL2013_AFN04_Fn4(INT8U *pBuf)
{
    INT8U i, time, ch;
    
    ch = pBuf[0];
    time = pBuf[2];
    
    uart_Answer_Yes();

    //  频率超过最大索引
    if(ch > 66)
    {
        ch %= 66;
    }

    //  发射频道
    sub_AX5043_set_test_mode(0x80 + ch);                                                
    for(i=0;i<time;i++)
    {
        drv_stm32WDT();
        drv_Delay10ms(100);
    }

    //  退出发射模式
    sub_AX5043_set_test_mode(0xff);
}

//===============================================================================================
//  函数名称:        DL2013_AFN05_Fn1
//  函数描述:   	    设置主节点地址
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           修改注释
//===============================================================================================
void DL2013_AFN05_Fn1(INT8U *pBuf)    
{
    INT8U i= 0;

    // 若主节点地址变动
    if(mCAC.aCAddr[0] != pBuf[0]
    ||mCAC.aCAddr[1] != pBuf[1]
    ||mCAC.aCAddr[2] != pBuf[2]
    ||mCAC.aCAddr[3] != pBuf[3]
    ||mCAC.aCAddr[4] != pBuf[4]
    ||mCAC.aCAddr[5] != pBuf[5])
    {
        // 提取CAC地址
        for(i=0; i < 6; i++)
        {
            mCAC.aCAddr[i] = pBuf[i];
        }

        // 计算panID
        LongToPanID(mCAC.aCAddr, mCAC.panID);

        // 计算实际工作频道
        mCAC.i1BigCH =(INT8U)((mCAC.panID[0]%(SUM_CH_NUM-1)) + 1);
        
        drv_setCH(mCAC.i1BigCH);
        
        // 需要根据底噪自动工作频道
        mCAC.bNewWrkCH  =   TRUE1;

        // 保存CAC变更信息
        cac_SaveCAC();  
    }
                                                                          	
    uart_Answer_Yes();                                                              
}

//===============================================================================================
//  函数名称:        DL2013_AFN05_Fn2
//  函数描述:   	    允许/禁止从节点上报
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           修改注释
//===============================================================================================
void DL2013_AFN05_Fn2(INT8U *pBuf) 
{
    if(pBuf[0]==1)
        mCAC.bReport=TRUE1;                                                                            //  允许上报
    else
        mCAC.bReport=FALSE0;                                                                           //  禁止上报
    uart_Answer_Yes();                                                                                      // 返回确认
}

//===============================================================================================
//  函数名称:        DL2013_AFN05_Fn3
//  函数描述:   	    启动广播
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                            <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     < 版 本 >      <描述>       
//			    陈炎       2013-09-30        1.0         修改注释
//===============================================================================================
void DL2013_AFN05_Fn3(INT8U *pBuf) 
{
    gDtMeter *tM = (gDtMeter*)(mRf.aBuf);	
    tM->i1Baud=(INT8U)(mCAC.i2BrdNum++);
    rf_BroadcastTime(&(pBuf[2]),pBuf[1]);                                       //  广播校时
    //01 12 68 99 99 99 99 99 99 68 08 06 37 53 44 48 34 4B 09
    drv_SetTimeMeter((INT16U)(xb_Capacity(mCAC.i2DownDAU)));                     //  动态规模

    mCAC.bMeter=TRUE1;
    mCAC.bBroad=TRUE1;	
    //uart_Answer_Guangbo(cac_GetBroadDelayTime(mCAC.i2ShiXi));                    // 返回确认
    uart_Answer_Guangbo(cac_GetBroadDelayTime(mCAC.i2AllNum+1));                    // 返回确认
}

//===============================================================================================
//  函数名称:        DL2013_AFN05_Fn4
//  函数描述:   	    设置从节点监控最大超时时间
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                            <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			    陈炎      2013-09-30           1.0           修改注释
//===============================================================================================
void DL2013_AFN05_Fn4(INT8U *pBuf)    
{
    mCAC.i1ReadMeterMaxTime = pBuf[0];	                                                         //  pBuf[0]存超时时间，单位s
    uart_Answer_Yes();                                                                           // 返回确认
}

//===============================================================================================
//  函数名称:        DL2013_AFN05_Fn5
//  函数描述:   	    设置无线通信参数
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :   
//  说明    :
//              1.    数据单元格式：pBuf[0]存信道组存测试通信速率，取值0~63；
//                           254自动选择，255保持不变，其余返回数据单元错误；
//                           pBuf[1]存无线主节点发射功率
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			      1.          陈 炎    2013-09-30           1.0           修改注释
//			      2.          陈 炎    2013-10-23           1.0           添加发射功率（底层暂未实现）
//===============================================================================================
void DL2013_AFN05_Fn5(INT8U *pBuf)
{
    // 功率只有四档
    if (pBuf[1]<=4)
    {
        mCAC.i1RFPower = pBuf[1];
    }
     
    // 自动选择或默认：不改变
    if(pBuf[0] >= 254)                                                                                //  254自动选择                                                    
    {
        uart_Answer_Yes();                                                                                        
    }
    // 1 ~ 32，更新保存频道
    else if((pBuf[0] <SUM_CH_NUM) && (pBuf[0] > 0))                                                 //  1~32
    {
        mCAC.i1BigCHXX = pBuf[0];
        cac_SaveCAC();
        uart_Answer_Yes();                                                                                         
    }
    else                                                                                              //  其余
    {
        uart_Answer_Err();
    }
}

//===============================================================================================
//  函数名称:        DL2013_AFN05_Fn100
//  函数描述:   	    设置场强门限
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :      
//  说明    :
//              1.    数据单元格式：pBuf[0]存场强门限，取值50~120，默认96或103；
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           生成函数
//===============================================================================================
void DL2013_AFN05_Fn100(INT8U *pBuf)
{
    if((pBuf[0] < MIN_RISS)||(pBuf[0] > MAX_RISS))
    {
        uart_Answer_Err();
    }
    else
    {
        mCAC.i1Valve = pBuf[0];
        cac_SaveCAC();	
        uart_Answer_Yes();                                                                                         // 返回确认
    }
}

//===============================================================================================
//  函数名称:        DL2013_AFN05_Fn101
//  函数描述:   	    设置中心节点时间
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :      
//  说明    :
//              1.    数据单元格式：pBuf[0]~pBuff[5]依次保存秒、分、时、日、月、年
//              2.       当前版本CAC系统时钟仅需保存时分秒信息，具体那一天不重要
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           生成函数
//===============================================================================================
void DL2013_AFN05_Fn101(INT8U *pBuf)
{
    gSysTime.count = 60 * pBuf[1] + pBuf[0];                                                    // 1h = 60min = 3600s
    gSysTime.hour = pBuf[2];                                                                               // 设置小时
    if((gSysTime.hour == 0x21)&&(TskOptimizeLimitTime==0))
    {
        drv_Printf("\n          晚上9点, 维护时间到: ");
        if(mCAC.bSetup != TRUE1 && nPrioIIFlag.fHourSate!=2)                                                                 // 有新增节点
        {
            nPrioIIFlag.fHourSate=2;
            drv_Printf("当前空闲, 启动维护... ");
            drv_SetTimeDown(10);
        }
        else
            drv_Printf("当前繁忙或已启动维护, 忽略...");
    }
    uart_Answer_Yes();                                                                                         // 返回确认
}

//===============================================================================================
//  函数名称:        DL2013_AFN10_Fn1
//  函数描述:   	    查询从节点数量
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           修改注释
//===============================================================================================
void DL2013_AFN10_Fn1(INT8U *pBuf)     
{
    pBuf[0]=(INT8U)(mCAC.i2DownDAU);                                                        //  从节点总数（下载总数）
    pBuf[1]=(INT8U)(mCAC.i2DownDAU>>8);
    pBuf[2]=(INT8U)(MAX_DAU);                                                                      //  路由支持最大从节点数量
    pBuf[3]=(INT8U)(MAX_DAU>>8);
    uart_Answer_Data(4);                                                                                  // 返回数据
    gFlgUrtCmpFile  =   TRUE1;
}

//===============================================================================================
//  函数名称:        DL2013_AFN10_Fn2
//  函数描述:   	    查询从节点信息
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    ：
//              1.    数据单元格式：pBuf[0]~pBuf[1]  从节点起始序号，指下载队列中的序号0~n-1
//                                 pBuf[2]           从节点数量，指要查询的从节点总数n
//              2.    注意两点：    一是下载队列节点按下载序一个个插入到DAU队列空位中
//                                 二是查询的从节点序号及数量均相对下载队列而言的
//              3.   830台体用到该接口，修改时请沟通高层软件组
//              4.   单帧最大可查询数量25
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			    陈炎    2013-09-30           1.0           添加注释
//===============================================================================================
void DL2013_AFN10_Fn2(INT8U *pBuf)     
{
    INT16U tN1;                                                                                               
    INT16U tN2;
    INT16U i;

    tN1	 = 1;
    tN2	 = (pBuf[1])<<8;                                                                                  //  tN2保存从节点起始序号
    tN2 += pBuf[0];                                                                                          //  此序号是下载序列的序号                                                                       

    for(i=0;i<mCAC.i2AllNum;i++)                                                                 // 遍历从节点队列
    {
        if(uart_DAUType(i,3)==TRUE1)                                                            //  若是下载节点，tN1加1
        {
            if(tN1 >= tN2)                                                                                    //  找到起始序号，跳出循环
                break;
            tN1++;                                                                                                
        }
    }
    tN2=0;
    for(;i<mCAC.i2AllNum;i++)                                                                      //  继续遍历从节点序列
    {
        if(uart_DAUType(i,3)==TRUE1)                                                            //  若是下载节点，tN2加1
        {
            pBuf[3+8*tN2]=mDAU[i].aDAddr[0];                                             //  写从节点地址
            pBuf[4+8*tN2]=mDAU[i].aDAddr[1];
            pBuf[5+8*tN2]=mDAU[i].aDAddr[2];
            pBuf[6+8*tN2]=mDAU[i].aDAddr[3];
            pBuf[7+8*tN2]=mDAU[i].aDAddr[4];
            pBuf[8+8*tN2]=mDAU[i].aDAddr[5];
            if(dauF_Good(i)==TRUE1)                                                              //  830台体接口:  0~6表中继层次，0x0f表不在网
            {
                pBuf[9+8*tN2]=mDAU[i].b4DLayerSon - 1;                                
            }
            else
            {
                pBuf[9+8*tN2]=15;                                                                       
            }
            pBuf[10+8*tN2]=(INT8U)(mDAU[i].aDProType[1]<<3);                                    // 协议类型:  档案类型
            tN2++;
            if(tN2>=pBuf[2] || tN2>=25)                                                         // 若tN2 达到25或要求查询数量，退出循环
                break;
        }
    }
    pBuf[0]=(INT8U)(mCAC.i2DownDAU);                                                            // 写返回帧从节点数量
    pBuf[1]=(INT8U)(mCAC.i2DownDAU>>8);
    pBuf[2]=(INT8U)tN2;                                                                         // 写返回帧从节点数量
    uart_Answer_Data(3+tN2*8);                                                                  // 返回数据

    if(FALSE0 == mCAC.bSetup && FALSE0 == mCAC.bOptimize && TRUE1  == gFlgUrtCmpFile)           // CAC未组网且未维护且开始档案同步                                                                                              
    {
        drv_Printf("\n本次下载%d个节点，其中有新增节点", pBuf[2]);
        drv_SetTimeDown(40);
    }
}

//===============================================================================================
//  函数名称:        DL2013_AFN10_Fn3
//  函数描述:   	    查询指定从节点上一级中继路由信息
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    ： 不支持
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           添加注释
//===============================================================================================
void DL2013_AFN10_Fn3(INT8U *pBuf)
{
    uart_Answer_Nonsup();                                                                               // 返回不支持
}

//===============================================================================================
//  函数名称:        DL2013_AFN10_Fn4
//  函数描述:   	    查询路由运行状态
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    ： 
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           添加注释
//                              陈炎    2014-02-08           1.1           添加自定义的【扩展】类字段
//===============================================================================================
void DL2013_AFN10_Fn4(INT8U *pBuf)
{
    //  写运行状态字 
    if(TRUE1 == mCAC.bSetup  || TRUE1 == mCAC.bOptimize || TRUE1 == mCAC.bScan)                    
    {
        pBuf[0] = 0;
    }
    else                                                                                                                   
    {
        pBuf[0] = 1;
    }
    if(FlagNew == TRUE1)                                                                            //  工作标志: 搜表写1
    {
        pBuf[0]+=2;
    }
    if(mCAC.bEvent == TRUE1)                                                                        //  上报事件标志: 有上报写1
        pBuf[0] += 4;

    //  写节点数量信息
    cac_CountDAU();                                                                                                                                                                                                                	
    pBuf[1]=(INT8U)(mCAC.i2DownDAU);                                                           //  下载节点
    pBuf[2]=(INT8U)(mCAC.i2DownDAU>>8);	
    pBuf[3]=(INT8U)(mCAC.i2GoodDAU);                                                            //  本次在网节点
    pBuf[4]=(INT8U)(mCAC.i2GoodDAU>>8);
    pBuf[5]=(INT8U)(mCAC.i2InNetDAU);	                                                           //  累计在网节点【扩展】
    pBuf[6]=(INT8U)(mCAC.i2InNetDAU>>8);

    //  写工作开关格式
    pBuf[7]=0;                                                                                                         
    if(mCAC.bYuLi == TRUE1)                                                                                 //  搜表
        pBuf[7] += 2;
    if(mCAC.bSetup == TRUE1)                                                                              //  组网
        pBuf[7] += 1;

    //  写路由学习步骤【扩展】
    //  0:  组网开始 1:  保存档案 2:  信标接收 3:  点名过程 
    //  4:  场强收集 5:  节点配置 6:  增补组网 7:  保留
    pBuf[8]     =   gFlgUrtCmpFile;
    pBuf[9]     =   0xFF;
    pBuf[10]    =   0;
    pBuf[11]    =   0;
    pBuf[12]    =   0;
    pBuf[13]    =   0;
    pBuf[14]    =   0;
    pBuf[15]    =   0;
    uart_Answer_Data(16);	                                                                                // 返回数据
}

//===============================================================================================
//  函数名称:        DL2013_AFN10_Fn5
//  函数描述:   	    查询未抄读成功从节点信息
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    ： 
//              1.   单帧最大可查询数量25
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3           1.0              生成函数
//===============================================================================================
void DL2013_AFN10_Fn5(INT8U *pBuf) 
{
    INT16U tN1;                                                                                               
    INT16U tN2;
    INT16U i;

    tN1	 = 1;
    tN2	 = (pBuf[1])<<8;                                                                                  //  tN2保存从节点起始序号
    tN2 += pBuf[0];                                                                                          //  此序号是未抄读成功序列的序号                                                                       

    for(i=0;i<mCAC.i2AllNum;i++)                                                                 // 遍历从节点队列
    {
        if(uart_DAUType(i,2)==TRUE1)                                                            //  若是不在网节点，tN1加1
        {
            if(tN1 >= tN2)                                                                                    //  找到起始序号，跳出循环
                break;
            tN1++;                                                                                                
        }
    }
    tN2=0;
    for(;i<mCAC.i2AllNum;i++)                                                                      //  继续遍历从节点序列
    {
        if(uart_DAUType(i,2)==TRUE1)                                                            //  若是不在网、故障节点，tN2加1
        {
            pBuf[3+8*tN2]=mDAU[i].aDAddr[0];                                                //  写从节点地址
            pBuf[4+8*tN2]=mDAU[i].aDAddr[1];
            pBuf[5+8*tN2]=mDAU[i].aDAddr[2];
            pBuf[6+8*tN2]=mDAU[i].aDAddr[3];
            pBuf[7+8*tN2]=mDAU[i].aDAddr[4];
            pBuf[8+8*tN2]=mDAU[i].aDAddr[5];
            pBuf[9+8*tN2]=0;
            pBuf[10+8*tN2]=0;                                                                           //  故障节点版本号写0
            tN2++;
            if(tN2>=pBuf[2] || tN2>=25)                                                           //  若tN2 达到25或要求查询数量，退出循环
                break;
        }
    }
    pBuf[0]=(INT8U)(mCAC.i2DownDAU);                                                    //  写返回帧从节点数量
    pBuf[1]=(INT8U)(mCAC.i2DownDAU>>8);
    pBuf[2]=(INT8U)tN2;                                                                                 //  写返回帧从节点数量
    uart_Answer_Data(3+tN2*8);                                                                  //  返回数据	
}

//===============================================================================================
//  函数名称:        DL2013_AFN10_Fn6
//  函数描述:   	    查询主动注册从节点信息
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    ： 
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3           1.0              生成函数
//===============================================================================================
void DL2013_AFN10_Fn6(INT8U *pBuf)
{
    INT16U tN1;                                                                                               
    INT16U tN2;
    INT16U i;

    tN1	 = 1;
    tN2	 = (pBuf[1])<<8;                                                                                  //  tN2保存从节点起始序号
    tN2 += pBuf[0];                                                                                          //  此序号是主动注册序列的序号                                                                       

    for(i=0;i<mCAC.i2AllNum;i++)                                                                 // 遍历从节点队列
    {
        if(uart_DAUType(i,5)==TRUE1)                                                            //  若是主动注册节点，tN1加1
        {
            if(tN1 >= tN2)                                                                                    //  找到起始序号，跳出循环
                break;
            tN1++;                                                                                                
        }
    }
    tN2=0;
    for(;i<mCAC.i2AllNum;i++)                                                                      //  继续遍历从节点序列
    {
        if(uart_DAUType(i,5)==TRUE1)                                                            //  若是主动注册节点，tN2加1
        {
            pBuf[3+8*tN2]=mDAU[i].aDAddr[0];                                                //  写从节点地址
            pBuf[4+8*tN2]=mDAU[i].aDAddr[1];
            pBuf[5+8*tN2]=mDAU[i].aDAddr[2];
            pBuf[6+8*tN2]=mDAU[i].aDAddr[3];
            pBuf[7+8*tN2]=mDAU[i].aDAddr[4];
            pBuf[8+8*tN2]=mDAU[i].aDAddr[5];
            if(dauF_Good(i)==TRUE1)                                                                    //  在网中继层次-1，不在网0x0F
            {
                pBuf[9+8*tN2]=mDAU[i].b4DLayerSon - 1;                                            //  写中继级别，其余位默认全0
            }
            else
            {
                pBuf[9+8*tN2]=15;                                                                       
            }
            pBuf[10+8*tN2]=(INT8U)(mDAU[i].aDProType[1]<<3);                    //  写版本号，其余位默认全0
            tN2++;
            if(tN2>=pBuf[2] || tN2>=25)                                                           //  若tN2 达到25或要求查询数量，退出循环
                break;
        }
    }
    pBuf[0]=(INT8U)(mCAC.i2DownDAU);                                                    //  写返回帧从节点数量
    pBuf[1]=(INT8U)(mCAC.i2DownDAU>>8);
    pBuf[2]=(INT8U)tN2;                                                                                 //  写返回帧从节点数量
    uart_Answer_Data(3+tN2*8);                                                                  //  返回数据	
}

//  查询路由中继信息
void DL2013_AFN10_Fn7(INT8U *pBuf)
{
    INT16U tDAUN=cac_CheckDAUNum(pBuf);
    INT8U  tL=0, tLen;

    if(tDAUN < MAX_DAU && dauF_In(tDAUN)==TRUE1)
    {
        INT8U pathNo;

        for (pathNo=0; pathNo<8; pathNo++)
        {
            if(pathNo==3 || pathNo==4 || pathNo==5)
                continue;
            pBuf[7+tL] = mth_3PathLayer(tDAUN, pathNo)-1;           // 中继级别
            if (pBuf[7+tL]>MAX_LAYER-1)
                pBuf[7+tL]=0;
            tLen = mth_3PathUart(tDAUN, pathNo, &pBuf[8+tL]);       // 中继节点
            pBuf[8+tL+tLen] = mth_3PathWeight(tDAUN, pathNo);       // 信任度
            tL += (tLen+2);
        }
        pBuf[6] = pathNo-3;                                         // 中继个数
        tL += 7;
    }
    else
    {
        pBuf[6]=0;

        tL=7;
    }

    // 返回数据
    uart_Answer_Data(tL);
}

//===============================================================================================
//  函数名称:        DL2013_AFN10_Fn100
//  函数描述:   	    查询网络规模
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    ：
//             1.        uart_DAUType(i,1)判定节点类型为微功率无线，目前暂用下载节点标准代替
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3           1.0              生成函数
//===============================================================================================
void DL2013_AFN10_Fn100(INT8U *pBuf)
{
    pBuf[0] = (INT8U)mCAC.i2Capacity;
    pBuf[1] = (INT8U)(mCAC.i2Capacity >> 8);
    uart_Answer_Data(2);	                                                      // 返回数据
    #ifdef  _LD_USE_TEST
        set_scan();
    #endif
}

//===============================================================================================
//  函数名称:        DL2013_AFN10_Fn101
//  函数描述:   	    查询微功率无线从节点信息
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :        
//  说明    ：
//             1.        uart_DAUType(i,1)判定节点类型为微功率无线，目前暂用下载节点标准代替
//             2.        软件版本号还需规约,该命令无数据单元
//  修改记录:        <作者>      <修改日期>     <版 本>      <描述>       
//			         陈 炎       2013-10-3        1.0        生成函数
//                   陈 炎       2014-05-18       1.1        解决队列非空、档案空时不上报bug（Down = 0 ，All>0） 
//===============================================================================================
void DL2013_AFN10_Fn101(INT8U *pBuf)
{
  INT16U i;                                                                     // 遍历索引（2B）
  INT16U tN2               =  0;                                                // 单帧个数计数
  INT16U li2MaxNum         =  mCAC.i2DownDAU;                                   // 档案总数
  INT16U li2LastNum        =  li2MaxNum;                                        // 剩余未上报总数

  for (i = 0; i < mCAC.i2AllNum; i++)                                           // 队列非空时，遍历队列
  {
    if (TRUE1 == uart_DAUType(i,6))                                             // 是微功率无线节点
    {
      pBuf[3+11*tN2]       =  mDAU[i].aDAddr[0];                                           
      pBuf[4+11*tN2]       =  mDAU[i].aDAddr[1];
      pBuf[5+11*tN2]       =  mDAU[i].aDAddr[2];
      pBuf[6+11*tN2]       =  mDAU[i].aDAddr[3];
      pBuf[7+11*tN2]       =  mDAU[i].aDAddr[4];
      pBuf[8+11*tN2]       =  mDAU[i].aDAddr[5];

#if   PRO_AREA == 0                                                              // 国网标准: 上报中继级别
      pBuf[9+11*tN2]       = (TRUE1 == dauF_Good(i)) ? mDAU[i].b4DLayerSon - 1 : 0x0f;
#elif PRO_AREA == 1                                                              // 北京标准：上报层次
      pBuf[9+11*tN2]       = (TRUE1 == dauF_Good(i)) ? mDAU[i].b4DLayerSon : 0;
#endif

      pBuf[10+11*tN2]      =  (INT8U)(mDAU[i].aDProType[1]<<3); 
      pBuf[11+11*tN2]      = (TRUE1 == dauF_Good(i)) ? mDAU[i].aDVersion[0]    : 0;
      pBuf[12+11*tN2]      = (TRUE1 == dauF_Good(i)) ? mDAU[i].aDVersion[1]    : 0;
      pBuf[13+11*tN2]      =  0;    
      
      tN2++;                                                                    // 节点计数tN2加1
    }

    if (tN2 >= li2LastNum  ||  tN2 >= 20)                                       // 若tN2满20或剩余总数，串口输出
    {
      pBuf[0]              =  (INT8U)(li2MaxNum);                               // 档案总数
      pBuf[1]              =  (INT8U)(li2MaxNum>>8);                            //
      pBuf[2]              =  (INT8U)tN2;                                       // 本帧应答总数（不超过20） 
      uart_Answer_Data(3+tN2*11);                                              
      drv_stm32WDT();                                                           // 喂狗
      drv_Delay10ms(50);
      li2LastNum           =  li2LastNum - tN2;                                 // 更新剩余总数
      tN2                  =  0;                                                // 计数tN2清空
    }

    if (0 == li2LastNum)                                                        // 剩余为0，可跳出遍历
    {
      break;
    }
  }

  if (0 == mCAC.i2AllNum)                                                       // 队列为空时
  {
    pBuf[0]                =  0;                                                // 档案总数写0
    pBuf[1]                =  0;                                                //
    pBuf[2]                =  0;                                                // 本帧上报数写0
    uart_Answer_Data(3+tN2*8);                                                  
  }
}

//===============================================================================================
//  函数名称:        DL2013_AFN11_Fn1
//  函数描述:   	    添加从节点
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                            <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        
//              1.    数据单元格式：pBuf[0]存从节点数量；其后以7B为单位连续存储，
//                                                            前6B存从节点地址，最后1B存该从节点通信协议类型
//              2.    从节点数目不大于30时，添加节点；超过30暂定为不响应
//
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           修改注释,加入断言
//===============================================================================================
void DL2013_AFN11_Fn1(INT8U *pBuf)    
{
    INT16U i;


    if(pBuf[0]<=30)                                                  
    {
        for(i=0;i<pBuf[0];i++)                                                                      // 循环添加DAU
        {
            cac_UserAddDAU(&(pBuf[1+i*7]));
        }
        cac_CountDAU();	                                                                            // 统计更新DAU数量信息
        
        if(FALSE0 == mCAC.bSetup && FALSE0 == mCAC.bOptimize)                                       // 当前未组网且未维护                                           
        {
            drv_Printf("\n本次下载%d个节点，其中有新增节点", pBuf[2]);
            drv_SetTimeDown(40);
        }

        uart_Answer_Yes();                                                                          // 确认应答	
    }
    else
    {
        uart_Answer_Err();
    }
}

//===============================================================================================
//  函数名称:        DL2013_AFN11_Fn2
//  函数描述:   	    删除从节点
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        
//              1.    数据单元格式：pBuf[0]存从节点数量；其后以6B为单位连续存储，
//                                                            6B存从节点地址
//              2.    从节点数目不大于30时，添加节点；超过30暂定为不响应
//
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           修改函数,加入断言
//===============================================================================================
void DL2013_AFN11_Fn2(INT8U *pBuf)  
{
    INT16U i;


    if(pBuf[0]<=30)
    {
        for(i=0;i<pBuf[0];i++)                                                                                 //  循环删除DAU
        {
            INT16U j;
            for(j=0;j<mCAC.i2AllNum;j++)                                                             //  遍历定位待删除DAU在序列中位置                                                    
            {                                                                                                                   
                if(mDAU[j].bDUse==TRUE1 
                    && mDAU[j].bDWrite==TRUE1
                    && Cb_CmpArry_stat(&(pBuf[1+i*6]),mDAU[j].aDAddr,6)==TRUE1)
                {
                    mDAU[j].bDUse=FALSE0;                                                                 //  使用标志位标记为0
                    mDAU[j].bDWrite=FALSE0;                                                              //  下载标志位标记为0
                    mth_ClearRiss(j);                                                                              //  清空场强表
                    break;                                                                                           
                }
            }
        }
        cac_CountDAU();	                                                                                    //  统计更新DAU数量信息
        uart_Answer_Yes();                                                                                   //  确认应答
    }
    else
    {
        uart_Answer_Err();
    }
}


//===============================================================================================
//  函数名称:        DL2013_AFN11_Fn3
//  函数描述:   	    设置从节点固定中继路径
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        不支持
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN11_Fn3(INT8U *pBuf) 
{
    uart_Answer_Nonsup();
}

//===============================================================================================
//  函数名称:        DL2013_AFN11_Fn4
//  函数描述:   	    设置工作模式
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        不支持
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN11_Fn4(INT8U *pBuf)
{
    uart_Answer_Nonsup();
}

//===============================================================================================
//  函数名称:        DL2013_AFN11_Fn5
//  函数描述:   	    激活从节点主动注册
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//
//  返回值  :        
//  说明    :        激活时重新组网并上报新发现节点信息
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-25           1.0           生成函数
//===============================================================================================
void DL2013_AFN11_Fn5(INT8U *pBuf)    
{	
#if MODE_DAU_COllECT
    INT16U i=0;
    for(i=0;i<MAX_DAU;i++)
    {
        mDAU[i].b2DOutNet=0;
    }
    uart_Answer_Yes();                                                                                         //  回复确认

    set_setup_all();
    FlagNew=TRUE1;
#else
    uart_Answer_Yes();                                                                                          //  回复确认
#endif
}

//===============================================================================================
//  函数名称:        DL2013_AFN11_Fn6
//  函数描述:   	    终止从节点主动注册
//  入口参数:	 
//
//  返回值  :        
//  说明    :        
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-12           1.0           生成函数
//===============================================================================================
void DL2013_AFN11_Fn6(INT8U *pBuf)    
{
#ifdef MODE_DAU_COllECT
    FlagNew = FALSE0;                                                                                           //  清标志位
    uart_Answer_Yes();                                                                                         //  回复确认
#else
    uart_Answer_Yes();                                                                                       //  回复确认
#endif
}


//===============================================================================================
//  函数名称:        DL2013_AFN11_Fn100
//  函数描述:   	    设置网络规模
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :      
//  说明    :
//              1.    数据单元格式：pBuf[1]~pBuf[0]存网络规模，取值2~512，默认256；
//              2.     原网络规模为DAU号码计算，现在修改为指令可设置，原细节还需修改
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-09-30           1.0           生成函数
//===============================================================================================
void DL2013_AFN11_Fn100(INT8U *pBuf)
{
    INT16U i;
    i = 256*pBuf[1] + pBuf[0];
    if((i < 2)||(i > 512))
    {
        uart_Answer_Err();
    }
    else
    {
        mCAC.i2Capacity = i;
        cac_SaveCAC();
        uart_Answer_Yes();                                                                                         // 返回确认
    }
}

//===============================================================================================
//  函数名称:        DL2013_AFN11_Fn101
//  函数描述:   	    启动网络维护进程
//  入口参数:	 
//  返回值  :      
//  说明    :
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3          1.0           生成函数
//===============================================================================================
void DL2013_AFN11_Fn101(INT8U *pBuf)
{
    // 若没有组网也没有网络维护
    if (FALSE0 == mCAC.bSetup && FALSE0 == mCAC.bOptimize)
    {
        // 倒数10s启动网络维护
        drv_SetTimeDown(10);
    }

    // 返回确认
    uart_Answer_Yes();                                                                                                                                                                                     
}

//===============================================================================================
//  函数名称:        DL2013_AFN11_Fn102
//  函数描述:   	    启动组网
//  入口参数:	 
//  返回值  :      
//  说明    :
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			    陈炎    2013-10-3          1.0           生成函数
//===============================================================================================
void DL2013_AFN11_Fn102(INT8U *pBuf)
{ 
    // 保存档案 
    cac_SaveAll();

    // 若没有组网
    if(FALSE0 == mCAC.bSetup)
    {
        // 需要组网
        set_setup_son();
    }

    // 返回确认
    uart_Answer_Yes();        
}

//===============================================================================================
//  函数名称:        DL2013_AFN12_Fn1
//  函数描述:   	    路由控制：重启
//  入口参数:	 
//  返回值  :      
//  说明    :
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3          1.0           生成函数
//===============================================================================================
void DL2013_AFN12_Fn1(INT8U *pBuf)    
{
    // 返回确认
    uart_Answer_Yes();
}

//===============================================================================================
//  函数名称:        DL2013_AFN12_Fn2
//  函数描述:   	    路由控制：暂停
//  入口参数:	 
//  返回值  :      
//  说明    :
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3          1.0           生成函数
//===============================================================================================
void DL2013_AFN12_Fn2(INT8U *pBuf) 
{
    // 返回确认
    uart_Answer_Yes();
}

//===============================================================================================
//  函数名称:        DL2013_AFN12_Fn3
//  函数描述:   	    路由控制：恢复
//  入口参数:	 
//  返回值  :      
//  说明    :
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3          1.0           生成函数
//===============================================================================================
void DL2013_AFN12_Fn3(INT8U *pBuf)   
{
    // 返回确认
    uart_Answer_Yes();
}

//===============================================================================================
//  函数名称:        DL2013_AFN13_Fn1
//  函数描述:   	    监控从节点
//  入口参数:	
//
//             1.  pDAU      <类型>  INT8U*
//                                 <说明>  字符串指针，指向从节点地址字段
//             2.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//             2.  pLen       <类型>  INT8U
//                                 <说明>  用户数据区的数据单元声明长度
//
//  返回值  :      
//  说明    :
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3          1.0                修改注释
//===============================================================================================
void DL2013_AFN13_Fn1(INT8U *pDAU,INT8U *pBuf,INT16U pLen)    
{
    INT8U tDataN = 0;
    INT16U tmpDAUNum = 0;
    INT8U tmpFrameType = 0;

    backMeter=1;

    //  数据长度异常
    tDataN=pBuf[2]*6+3;                                                                                   //  计算报文长度L字段下标
    if( (pLen > LEN_USER)                                                                                   //  声明长度超过定义最大值
        || (pBuf[tDataN]+pBuf[2]*6+4>pLen))                                                      //  数据单元实际长度大于声明长度
    {
        drv_Printf("\n数据长度错误");
        uart_Answer_Err();
        return;
    }

    //  已有抄表任务
    if(mCAC.bMeter==TRUE1)                                                            
    {
        drv_Printf("\n已有抄表任务");
        return ;                                                                //  返回主节点忙
    }

    //  查找DAU序号
    tmpDAUNum = cac_CheckDAUNum(pDAU);
    drv_Printf("\n目标DAU[%3d] =  ",tmpDAUNum);
    drv_PrintfDAU(pDAU);

    //  节点不在档案
    if(tmpDAUNum >= MAX_DAU)                                                                                                    
    {
        drv_Printf("\n节点不在档案");
        uart_Answer_Unknown();                                                                            //  返回不在网
        return ;
    }

    //  若为透传命令
    if(pBuf[0] == 0x00)
    {
        //  更新DAU协议类型
        mDAU[tmpDAUNum].aDProType[0]= Get645FrameType(&(pBuf[tDataN+1]));
    }
    //  若有协议字段
    else
    {
        //  解析报文帧类型
        tmpFrameType = Get645FrameType(&(pBuf[tDataN+1]));
        mDAU[tmpDAUNum].aDProType[0] = tmpFrameType;
    }

    //  延时通信相关   
    if(pBuf[1]==1)                                                                                                
    {                                                                                                                      
        INT8U i;	

        //  回复14H-F3上行
        uartTxBuf[0]=pDAU[0];
        uartTxBuf[1]=pDAU[1];
        uartTxBuf[2]=pDAU[2];
        uartTxBuf[3]=pDAU[3];
        uartTxBuf[4]=pDAU[4];
        uartTxBuf[5]=pDAU[5];
        uartTxBuf[6]=YanShiCanShu(pDAU);
        uartTxBuf[7]=0;
        uartTxBuf[8]=pBuf[tDataN];
        for(i=0;i<uartTxBuf[8];i++)
        {
            uartTxBuf[9+i] = pBuf[tDataN+i+1];
        }
        uart_ReportData((INT8U*)0, uartTxBuf, pBuf[tDataN]+9, 0x14, 3);

        //  保存序号 延缓抄表
        drv_Printf("\n======================延时相关 暂缓抄表======================");
        gDAUNum = tmpDAUNum;

    }	
    //  非延时通信相关  调用set_Meter设置抄表
    else                                                                                                                     
    {
        drv_Printf("\n==============================启动抄表=============================");
        flag_at_meter = 0;
        set_Meter(tmpDAUNum,&(pBuf[tDataN+1]),pBuf[tDataN],mDAU[tmpDAUNum].aDProType[0],0,RF_ASK_METER_DATA);
    }  
}

void DL2013_AFN13_Fn101(INT8U *pDAU,INT8U *pBuf,INT16U pLen)    
{
    INT8U tDataN = 0;
    INT16U tmpDAUNum = 0;
    INT8U tmpFrameType = 0;
    
    backMeter=3;

  
    //  数据长度异常
    tDataN=pBuf[2]*6+3;                                                                                   //  计算报文长度L字段下标
    if( (pLen > LEN_USER)                                                                                   //  声明长度超过定义最大值
        || (pBuf[tDataN]+pBuf[2]*6+4>pLen))                                                      //  数据单元实际长度大于声明长度
    {
        drv_Printf("\n数据长度错误");
        uart_Answer_Err();
        return;
    }

    //  已有抄表任务
    if(mCAC.bMeter==TRUE1)                                                            
    {
        drv_Printf("\n已有抄表任务");
        return ;                                                                //  返回主节点忙
    }

    //  查找DAU序号
    tmpDAUNum = cac_CheckDAUNum(pDAU);
    drv_Printf("\n目标DAU[%3d] =  ",tmpDAUNum);
    drv_PrintfDAU(pDAU);

    //  节点不在档案
    if(tmpDAUNum >= MAX_DAU)                                                                                                    
    {
        drv_Printf("\n节点不在档案");
        uart_Answer_Unknown();                                                                            //  返回不在网
        return ;
    }

    //  若为透传命令
    if(pBuf[0] == 0x00)
    {
        //  更新DAU协议类型
        mDAU[tmpDAUNum].aDProType[0]= Get645FrameType(&(pBuf[tDataN+1]));
    }
    //  若有协议字段
    else
    {
        //  解析报文帧类型
        tmpFrameType = Get645FrameType(&(pBuf[tDataN+1]));
        mDAU[tmpDAUNum].aDProType[0] = tmpFrameType;
    }
    
    //  延时通信相关
    if(pBuf[1]==1)
    {
        INT8U i;
        
        //  回复14H-F3上行
        uartTxBuf[0]=pDAU[0];
        uartTxBuf[1]=pDAU[1];
        uartTxBuf[2]=pDAU[2];
        uartTxBuf[3]=pDAU[3];
        uartTxBuf[4]=pDAU[4];
        uartTxBuf[5]=pDAU[5];
        uartTxBuf[6]=YanShiCanShu(pDAU);
        uartTxBuf[7]=0;
        uartTxBuf[8]=pBuf[tDataN];
        for(i=0;i<uartTxBuf[8];i++)
        {
            uartTxBuf[9+i] = pBuf[tDataN+i+1];
        }
        uart_ReportData((INT8U*)0, uartTxBuf, pBuf[tDataN]+9, 0x14, 3);
        
        //  保存序号 延缓抄表
        drv_Printf("\n======================延时相关 暂缓抄表======================");
        gDAUNum = tmpDAUNum;
    }
    //  非延时通信相关  调用set_Meter设置抄表
    else                                                                                                                     
    {
        drv_Printf("\n==============================启动抄表=============================");
          flag_at_meter = 0;
        set_Meter(tmpDAUNum,&(pBuf[tDataN+1]),pBuf[tDataN],mDAU[tmpDAUNum].aDProType[0],0,RF_ASK_DLMS_DATA);
        
    }
}

//===============================================================================================
//  函数名称:        DL2013_AFN14_Fn1
//  函数描述:   	    路由请求抄读内容
//  入口参数:	 
//  返回值  :      
//  说明    :       暂不支持,14H为主动请求的，响应时只配置不应答
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3          1.0                生成函数
//===============================================================================================
void DL2013_AFN14_Fn1(INT8U *pBuf)
{
    ;
}


//===============================================================================================
//  函数名称:        DL2013_AFN14_Fn2
//  函数描述:   	    路由请求集中器时钟
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :      
//  说明    :        14H为主动请求的，响应时只配置不应答
//              1.    数据单元格式：pBuf[0]~pBuff[5]依次保存秒、分、时、日、月、年
//              2.       当前版本CAC系统时钟仅需保存时分秒信息，具体那一天不重要
//             
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			    陈炎    2013-10-3         1.0         生成函数
//              陈炎    2013-10-18        1.0         删除返回确认，14H为上行请求，不应答
//===============================================================================================
void DL2013_AFN14_Fn2(INT8U *pBuf)
{
    gSysTime.count = 60 * pBuf[1] + pBuf[0];                                                    // 1h = 60min = 3600s
    gSysTime.hour = pBuf[2];                                                                    // 设置小时

    if (gSysTime.hour==0x21)
    {
        drv_Printf("\n          晚上9点, 维护时间到: ");
        if(mCAC.bSetup != TRUE1 && nPrioIIFlag.fHourSate!=2)                                    // 有新增节点
        {
            nPrioIIFlag.fHourSate=2;
            drv_Printf("当前空闲, 启动维护... ");
            drv_SetTimeDown(40);
        }
        else
            drv_Printf("当前繁忙或已启动维护, 忽略...");
    }
}

//===============================================================================================
//  函数名称:        DL2013_AFN14_Fn3
//  函数描述:   	    请求通信延时修正通信数据
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :      
//  说明    :        14H为主动请求的，响应时只配置不应答
//              1.    数据单元格式：pBuf[0]保存数据长度L，pBuf[1]~pBuf[L]保存修正通信数据报文
//                                                             L为0表示放弃本次通信
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-3           1.0           生成函数
//===============================================================================================
void DL2013_AFN14_Fn3(INT8U *pBuf)
{
    if(pBuf[0] == 0)                                                                                                //  L = 0 ，放弃本次通信
        return;
    backMeter=1;
    drv_Printf("\n==============================启动抄表=============================");
    set_Meter(gDAUNum,&(pBuf[1]),pBuf[0],mDAU[gDAUNum].aDProType[0],0,RF_ASK_METER_DATA);                                    
}

//===============================================================================================
//  函数名称:        DL2013_AFN15_Fn1
//  函数描述:   	    文件传输
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                            <说明>  字符串指针，指向用户数据区的数据单元
//              
//  返回值  :      
//  说明    :         
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			    陈炎       2013-10-4       1.0         添加注释
//===============================================================================================
void DL2013_AFN15_Fn1(INT8U *pBuf)
{
    INT16U tN=0;
    INT16U tL=0;
    INT16U tD=0;
    INT32U tALL=0;
    INT16U tS=0;
    INT8U i=0;
    INT8U flgRest=0;

    tD=pBuf[4];                                                                                               //  pBuf[3-4] 存总段数n
    tD=tD<<8;
    tD+=pBuf[3];

    tN=pBuf[6];                                                                                              //  pBuf[5-8] 存当前段号i
    tN=tN<<8;
    tN+=pBuf[5];

    tL=pBuf[10];                                                                                            //  pBuf[9-10] 存当前段长度L
    tL=tL<<8;                                                                                                 //  除最后一段，其余长度固定
    tL+=pBuf[9];

    if(pBuf[1]==1)                                                                                          //  pBuf[1] 文件属性：结束帧为01H，其余为00H
        flgRest=1;

    if(tL==0)                                                                                                  //  若为最后一段
    {	
        pBuf[0]=(INT8U)tN;                                                                           
        pBuf[1]=(INT8U)(tN>>8);
        pBuf[2]=0;
        pBuf[3]=0;
        uart_Answer_Data(4);                                                                      //  返回数据

        if(flgRest==1)                                                                                      //  最后一段到来
        {
            drv_Delay10ms(20);
            drv_Resetcac();                                                                              //  重启CAC，升级
        }
        return;
    }

    if(tL>=256)                                                                                             //  文件超过最大长度                                                                                      
    {	
        pBuf[0]=0xFF;                                                                                    //  返回0xFFFFH报错
        pBuf[1]=0xFF;
        pBuf[2]=0;
        pBuf[3]=0;
        uart_Answer_Data(4);
        return;
    }

    tALL=tN*tL;                                                                                             //  计算收到字节，tN存当前段号，tL存段长
    tS=(INT16U)tALL/128;                                                                           //  找到起始页	
    i=tALL%128;                                                                                            //  找到起始位	
    drv_write_program(&pBuf[11],i,tS+1);                                               //  第1页	
    if(tL>128)                                                                                                 //  第2页
        drv_write_program(&pBuf[11+128-i],0,tS+2);                                     
    if(tN==0)                                                                                                   //  如果是第 1 页需要写第 0 页
    {
        tALL=tD*tL;
        tS=(INT16U)(tALL/128+1);
        pBuf[0]=0xA1;
        pBuf[1]=(INT8U)tS;
        pBuf[2]=(INT8U)(tS>>8);
        drv_write_program(pBuf,0,0xFFFF);
    }	
    pBuf[0]=(INT8U)tN;                                                                                      //  返回数据
    pBuf[1]=(INT8U)(tN>>8);
    pBuf[2]=0;
    pBuf[3]=0;
    uart_Answer_Data(4);

    if(flgRest==1)
    {
        drv_Delay10ms(20);
        drv_Resetcac();
    }
}

//  读取场强 0A 01 0A
void DL2013_AFN0A_Fn51(INT8U *pBuf)
{
    INT16U tDAUN;
    INT16U i;
    INT8U k;
    INT8U tPage;
    INT8U *pRiss;

    tDAUN=pBuf[1];
    tDAUN=tDAUN<<8;
    tDAUN+=pBuf[0];

    if(tDAUN==0xFFFF)
        pRiss=drvBuf.pRiss;
    else
        pRiss=drvBuf.pRiss+(tDAUN+1)*LEN_ONE_RISS;

    tPage=1;
    k=0;
    for(i=0;i<=MAX_DAU;i++)
    {
        if((i&1)==0)
            pBuf[4+k]=(*pRiss)>>4;
        else
        {
            pBuf[4+k]=(*pRiss)&0x0F;
            pRiss++;
        }

        k++;
        if(k>=200)
        {
            pBuf[2]=tPage;
            pBuf[3]=MAX_DAU/200+1;

            // 返回数据
            uart_Answer_Data(204);

            // 再等待100ms
            drv_Delay10ms(30);	

            k=0;
            tPage++;
        }
    }
    // 最后一帧
    if(k>0)
    {
        pBuf[2]=tPage;
        pBuf[3]=MAX_DAU/200+1;

        for(;k<200;k++)
            pBuf[4+k]=0;

        // 返回数据
        uart_Answer_Data(204);
    }
}

// 读取档案    0A 02 0A
void DL2013_AFN0A_Fn52(INT8U *pBuf)
{
    INT8U tN;
    INT16U i;
    INT16U tTop=0xFFFF;
    INT16U tEnd=0xFFFF;

    tN=0;
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(tTop==0xFFFF)
            tTop=i;

        tEnd=i;

        // 读取有效点
        if(dauF_Down(i)==TRUE1)
        {
            pBuf[4+6*tN]=mDAU[i].aDAddr[0];
            pBuf[5+6*tN]=mDAU[i].aDAddr[1];
            pBuf[6+6*tN]=mDAU[i].aDAddr[2];
            pBuf[7+6*tN]=mDAU[i].aDAddr[3];
            pBuf[8+6*tN]=mDAU[i].aDAddr[4];
            pBuf[9+6*tN]=mDAU[i].aDAddr[5];
        }
        else
        {
            pBuf[4+6*tN]=0;
            pBuf[5+6*tN]=0;
            pBuf[6+6*tN]=0;
            pBuf[7+6*tN]=0;
            pBuf[8+6*tN]=0;
            pBuf[9+6*tN]=0;
        }


        tN++;
        if(tN>=30)
        {
            pBuf[0]=(INT8U)tTop;
            pBuf[1]=(INT8U)(tTop>>8);
            pBuf[2]=(INT8U)tEnd;
            pBuf[3]=(INT8U)(tEnd>>8);

            // 返回数据
            uart_Answer_Data(184);

            // 再等待100ms
            drv_Delay10ms(30);	

            tN=0;

            tTop=0xFFFF;
        }
    }

    if(tN!=0)
    {
        pBuf[0]=(INT8U)tTop;
        pBuf[1]=(INT8U)(tTop>>8);
        pBuf[2]=(INT8U)tEnd;
        pBuf[3]=(INT8U)(tEnd>>8);

        // 返回数据
        uart_Answer_Data(4+tN*6);

        // 再等待100ms
        drv_Delay10ms(30);	
    }

    // 结束发送
    pBuf[0]=0;
    pBuf[1]=0;
    pBuf[2]=0;
    pBuf[3]=0;

    // 返回数据
    uart_Answer_Data(4);
}

// 读取底噪
void DL2013_AFN0A_Fn54(INT8U *pBuf)
{
    INT16U tDAUN;
    INT8U tL;

    tDAUN=pBuf[1];
    tDAUN=tDAUN<<8;
    tDAUN+=pBuf[0];

    // 读CAC底噪
    if(tDAUN==0xFFFF)
    {
        pBuf[2] = mCAC.aCBGNoise[0];
        pBuf[3] = mCAC.aCBGNoise[1];
        pBuf[4] = mCAC.aCBGNoise[2];
        pBuf[5] = mCAC.aCBGNoise[3];
        tL = 6;
    }
    else
    {
        Co_CpyArry_stat(mDAU[tDAUN].aDBGNoise, &pBuf[2], 66);
        tL = 68;
    }
    uart_Answer_Data(tL);
}

// 读取路由信道遍历
extern INT16U FC_Path[MAX_LAYER*2];
void DL2013_AFN0A_Fn55(INT8U *pBuf)
{
    INT16U tDAUN;
    INT8U j, pathNo, tL;

    tDAUN=pBuf[1];
    tDAUN=tDAUN<<8;
    tDAUN+=pBuf[0];

    pBuf[2] = 4;
    for(pathNo=3; pathNo<7; pathNo++)
    {
        pBuf[3] = pathNo-3;

        if(mth_3PathWeight(tDAUN, pathNo))
        {
            INT8U tLayer = mth_3PathLayer(tDAUN, pathNo);

            pBuf[4] = tLayer+1;
            // 拷贝路径
            Co_SetArry_stat(&pBuf[5], 0xAA, 6);
            mth_3PathGet(FC_Path, tDAUN, pathNo);
            for(j=0; j<tLayer-1; j++)
                Co_CpyArry_stat(mDAU[FC_Path[j]].aDAddr, &pBuf[11+6*j], 6);
            Co_CpyArry_stat(mDAU[tDAUN].aDAddr, &pBuf[11+6*j], 6);

            // 拷贝场强
            Co_SetArry_stat(&pBuf[5+6*(tLayer+1)], 0x00, 8*(tLayer+1));
            // 遍历信息有效
            if(BYTE_READ_BIT(mDAU[tDAUN].bDScanRssi, pathNo))
                Co_CpyArry_stat(mDAU[tDAUN].aScanRssi[pathNo-3], &pBuf[9+6*(tLayer+1)], 8*tLayer);

            // 返回数据
            tL = 5+14*(tLayer+1);
            uart_Answer_Data(tL);
            drv_Delay10ms(30);
        }
        else
        {
            // 当前路径节点数
            pBuf[4] = 0;

            // 返回数据
            tL = 5;
            uart_Answer_Data(tL);
            drv_Delay10ms(30);
        }
    }
}

// 读取上下不同径
void DL2013_AFN0A_Fn56(INT8U *pBuf)
{
    INT16U tDAUN;
    INT8U i, j, pathNo, tL;

    tDAUN=pBuf[1];
    tDAUN=tDAUN<<8;
    tDAUN+=pBuf[0];
    
    for (pathNo=8; pathNo<12; pathNo+=2)
    {
        if (mth_3PathWeight(tDAUN, pathNo))
        {
            INT8U tLayer, tLayerUp;
    
            // 下行路径
            mth_3PathGet(FC_Path, tDAUN, pathNo);
            tLayer = mth_3PathLayer(tDAUN, pathNo);
            pBuf[2] = tLayer;
            for (i=0; i<tLayer-1; i++)
                Co_CpyArry_stat(mDAU[FC_Path[i]].aDAddr, &pBuf[3+6*i], 6);
    
            // 上行路径
            mth_3PathGet(FC_Path, tDAUN, pathNo+1);
            tLayerUp = mth_3PathLayer(tDAUN, pathNo+1);
            pBuf[3+6*i] = tLayerUp;
            for (j=0; j<tLayerUp-1; j++)
                Co_CpyArry_stat(mDAU[FC_Path[j]].aDAddr, &pBuf[4+6*i], 6);
    
            tL = 4+6*(tLayer+tLayerUp-2);
        }
        else
        {
            // 下行路径
            pBuf[2] = 0;
            // 上行路径
            pBuf[3] = 0;
    
            tL = 4;
        }
        
        uart_Answer_Data(tL);
        drv_Delay10ms(30);
    }
}

void DL2013_AFNF0_Fn20(INT8U *pBuf)
{
    //  写运行状态字 
    if(TRUE1 == mCAC.bSetup  ||  TRUE1 == mCAC.bOptimize)                    //  路由完成标志: 完成写1 
    {
        pBuf[0] = 0;
    }
    else                                                                                                                   
    {
        pBuf[0] = 1;
    }
    if(FlagNew == TRUE1)                                                                                        //  工作标志: 搜表写1
    {
        pBuf[0]+=2;
    }
    if(mCAC.bEvent == TRUE1)                                                                               //  上报事件标志: 有上报写1
        pBuf[0] += 4;

    //  写节点数量信息
    cac_CountDAU();                                                                                                                                                                                                                	
    pBuf[1]=(INT8U)(mCAC.i2DownDAU);                                                           //  下载节点
    pBuf[2]=(INT8U)(mCAC.i2DownDAU>>8);	
    pBuf[3]=(INT8U)(mCAC.i2GoodDAU);                                                            //  本次在网节点
    pBuf[4]=(INT8U)(mCAC.i2GoodDAU>>8);
    pBuf[5]=(INT8U)(mCAC.i2InNetDAU);	                                                           //  累计在网节点【扩展】
    pBuf[6]=(INT8U)(mCAC.i2InNetDAU>>8);

    //  写工作开关格式
    pBuf[7]=0;                                                                                                         
    if(mCAC.bYuLi == TRUE1)                                                                                 //  搜表
        pBuf[7] += 2;
    if(mCAC.bSetup == TRUE1)                                                                              //  组网
        pBuf[7] += 1;

    //  写路由学习步骤【扩展】
    //  0:  组网开始 1:  保存档案 2:  信标接收 3:  点名过程 
    //  4:  场强收集 5:  节点配置 6:  增补组网 7:  保留
    pBuf[8]     =   gFlgUrtCmpFile;
    pBuf[9]     =   0xFF;
    pBuf[10]    =   0;
    pBuf[11]    =   0;
    pBuf[12]    =   0;
    pBuf[13]    =   0;
    pBuf[14]    =   0;
    pBuf[15]    =   0;
    uart_Answer_Data(16);	                                                                                // 返回数据
}

void DL2013_AFNF0_Fn1(INT8U *pBuf)  // 读取CAC状态信息
{
    if(mCAC.bSetup==TRUE1)
		pBuf[0]=2;
	else if(mCAC.bMeter==TRUE1)
		pBuf[0]=5;
	else
		pBuf[0]=1;

	cac_CountDAU();

	// 组网节点个数
	pBuf[1]=(INT8U)(mCAC.i2InNetDAU);
	pBuf[2]=(INT8U)(mCAC.i2InNetDAU>>8);
	// 下载节点个数
	pBuf[3]=(INT8U)(mCAC.i2DownDAU);
	pBuf[4]=(INT8U)(mCAC.i2DownDAU>>8);
	// 故障节点个数
	pBuf[5]=(INT8U)(mCAC.i2BadDAU);
	pBuf[6]=(INT8U)(mCAC.i2BadDAU>>8);

	// 返回数据
	uart_Answer_Data(7);
}

void DL2013_AFNF0_Fn2(INT8U *pBuf)  // 读取DAU状态信息（多次读取）
{
    INT16U tN1;
    INT16U tN2;
    INT16U i;

    tN1	 = 0;
    tN2	 = (pBuf[2])<<8;
    tN2 += pBuf[1];

    // 找到头序号
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(uart_DAUType(i,pBuf[0])==TRUE1)
        {
            if(tN2<=tN1)
                break;

            tN1++;
        }
    }
    tN2=0;
    // 读取数据
    for(;i<mCAC.i2AllNum;i++)
    {
        if(uart_DAUType(i,pBuf[0])==TRUE1)
        {
            pBuf[4+7*tN2]=mDAU[i].aDAddr[0];
            pBuf[5+7*tN2]=mDAU[i].aDAddr[1];
            pBuf[6+7*tN2]=mDAU[i].aDAddr[2];
            pBuf[7+7*tN2]=mDAU[i].aDAddr[3];
            pBuf[8+7*tN2]=mDAU[i].aDAddr[4];
            pBuf[9+7*tN2]=mDAU[i].aDAddr[5];

            switch(pBuf[0])
            {
            case 0:
                pBuf[10+7*tN2]=mDAU[i].b4DLayerSon;
                break;

            case 3:
                pBuf[10+7*tN2]=mDAU[i].aDVersion[0];
                break;

            default:
                pBuf[10+7*tN2]=0;
                break;
            }

            tN2++;
            if(tN2>=pBuf[3] || tN2==30)
                break;
        }
    }

    pBuf[3]=(INT8U)tN2;

    // 返回数据
    uart_Answer_Data(4+tN2*7);
}

//===============================================================================================
//  函数名称:        DL2013_AFNF0_Fn3
//  函数描述:   	    读取DAU状态信息（一次读取）
//  入口参数:	 
//
//             1.  pBuf    <类型>  INT8U *                     取值范围0 ~ n，n为DAU序列下标最大值
//                              <说明>   指向数据单元                                                                 
//  返回值  :           TRUE1 表示是，FALSE0表示不是   
//  说明    ：内部测试命令
//             pBuf[0]保存读取DAU类型    0     在网             命令 F0 04 00 00
//                                                                              1     微功率无线       命令 F0 04 00 01 
//                                                                              2     故障             命令 F0 04 00 02
//                                                                              3     下载             命令 F0 04 00 03
//                                                                              5     发现                                                  
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			                     陈炎    2013-10-10           1.0           添加注释
//===============================================================================================
void DL2013_AFNF0_Fn3(INT8U *pBuf)  
{
    INT8U tF;
    INT8U tN;
    INT16U i;

    tF=FALSE0;
    tN=0;
    for(i=0;i<mCAC.i2AllNum;i++)
    {
        if(uart_DAUType(i,pBuf[0])==TRUE1)
        {
            pBuf[4+7*tN]=mDAU[i].aDAddr[0];
            pBuf[5+7*tN]=mDAU[i].aDAddr[1];
            pBuf[6+7*tN]=mDAU[i].aDAddr[2];
            pBuf[7+7*tN]=mDAU[i].aDAddr[3];
            pBuf[8+7*tN]=mDAU[i].aDAddr[4];
            pBuf[9+7*tN]=mDAU[i].aDAddr[5];

            switch(pBuf[0])
            {
            case 0:                                                                              //  在网
                pBuf[10+7*tN]=mDAU[i].b4DLayerSon;                   //  写层次号
                break;

            case 3:                                                                              //  下载
                pBuf[10+7*tN]=mDAU[i].aDVersion[0];                      //  写版本类型
                break;

            default:                                                                              //  其余
                pBuf[10+7*tN]=0;                                                         // 写0
                break;
            }

            tN++;
            if(tN>=30)
            {
                pBuf[3]=30;

                // 返回数据
                uart_Answer_Data(214);

                // 再等待100ms
                drv_Delay10ms(30);	

                tF=TRUE1;
                tN=0;
            }
        }
    }

    if(tN!=0 || tF==FALSE0)
    {
        pBuf[3]=tN;

        // 返回数据
        uart_Answer_Data(4+tN*7);
    }
}
void DL2013_AFNF0_Fn4(INT8U *pBuf)  // 启动组网进程
{
    ;
}
void DL2013_AFNF0_Fn5(INT8U *pBuf)  // 启动全网点名进程
{
    // 返回确认
    uart_Answer_Yes();

    //set_call_all();
}

void DL2013_AFNF0_Fn6(INT8U *pBuf)  // 启动网络维护进程
{
    // 返回确认
    uart_Answer_Yes();

    //set_call_all();
}

void DL2013_AFNF0_Fn7(INT8U *pBuf)  // 进入出厂检查模式
{
    uart_Answer_Yes();
}
void DL2013_AFNF0_Fn8(INT8U *pBuf)  // 进入送检测试模式
{
    uart_Answer_Yes();
}
void DL2013_AFNF0_Fn9(INT8U *pBuf)  // 备用
{
    // 返回不支持
    uart_Answer_Nonsup();
}
void DL2013_AFNF0_Fn10(INT8U *pBuf)  // 备用
{
    uart_Answer_Nonsup();
}

//===============================================================================================
//  函数名称:        DL2013_AFNF0_Fn12
//  函数描述:   	    查询厂商代码和版本信息
//  入口参数:	 
//
//             1.  pBuf       <类型>  INT8U*
//                                 <说明>  字符串指针，指向用户数据区的数据单元   
//  返回值  :        
//  说明    :        
//             1.
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			    陈炎       2013-09-25      1.0         生成函数
//===============================================================================================
void DL2013_AFNF0_Fn12(INT8U *pBuf) 
{
    
}

void DL2013_AFNF0_Fn100(INT8U *pBuf)  // CAC验证使用
{
    INT32U tCRC;

    tCRC=LZCRC32(pBuf,0,4);

    pBuf[0]=(INT8U)(tCRC>>0);
    pBuf[1]=(INT8U)(tCRC>>8);
    pBuf[2]=(INT8U)(tCRC>>16);
    pBuf[3]=(INT8U)(tCRC>>24);

    // 返回数据
    uart_Answer_Data(4);
}
void DL2013_AFNF0_Fn101(INT8U *pBuf)  // 读取内部版本号
{
    pBuf[0] = DT_PROTOCOL;
    pBuf[1] = DT_EDITIOM_H;
    pBuf[2] = DT_EDITIOM_L;
    pBuf[3] = DT_CUSTOM;

    // 返回数据
    uart_Answer_Data(4);
}

void DL2013_AFNF0_Fn102(INT8U *pBuf)  // 读取CAC真实频道号
{
    pBuf[0] = 0xAA;
    pBuf[1] = mCAC.i1BigCH;
    pBuf[2] = 0xAA;

    // 返回数据
    uart_Answer_Data(3);
}
void DL2013_AFNF0_Fn103(INT8U *pBuf)  // 备用
{

}

//===============================================================================================
//  函数名称:        DL2013_AFNF0_Fn104
//  函数描述:   	    打印DAU队列信息
//  入口参数:	   
//  返回值  :        
//  说明    :        VS调试使用
//             1.
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			    陈炎       2013-11-18       1.0        生成函数
//===============================================================================================
void DL2013_AFNF0_Fn104(INT8U *pBuf)  // 备用
{
}

//===============================================================================================
//  函数名称:        DL2013_AFNF0_Fn105
//  函数描述:   	    打印Meter队列信息
//  入口参数:	   
//  返回值  :        
//  说明    :        VS调试使用
//             1.
//  修改记录:  <作者>      <修改日期>     <版本 >      <描述>       
//			   陈炎       2013-11-18           1.0         生成函数
//===============================================================================================
// 读第一层信道遍历
void DL2013_AFNF0_Fn105(INT8U *pBuf)  // 备用
{
    INT8U tPageNow;
    INT8U tPageAll = (mCAC.i2Neighbor>>4) + 1;
    INT8U i;

    if( (mCAC.i2Neighbor&0x000F)==0 )
        tPageAll--;

    pBuf[0] = tPageAll;

    // 无第一层信道数据
    if(tPageAll==0)
    {
        pBuf[1] = 0;
        pBuf[2] = 0;
        uart_Answer_Data(3);
    }

    for(tPageNow=0; tPageNow<tPageAll; tPageNow++)
    {
        INT16U tTop = tPageNow<<4;
        INT16U tEnd = tTop;

        for(i=0; i<16; i++)
        {
            if(tEnd<mCAC.i2Neighbor)
            {
                Co_CpyArry_stat(mNeighbor[tEnd].aNAddr, &pBuf[3+14*i], 6);
                Co_CpyArry_stat(mNeighbor[tEnd].aNRssi, &pBuf[9+14*i], 8);
                tEnd++;
            }
            else
            {
                break;
            }
        }
        pBuf[1] = tPageNow;
        pBuf[2] = tEnd-tTop;

        uart_Answer_Data(3+14*(tEnd-tTop));
        drv_Delay10ms(30);
    }
}

// 读取路径信息
// 0:读取0~4条路径
// 1：读取5~9条路径信息
// 2: 读取10~12条路径信息
// 3：读取当前使用路径信息
void DL2013_AFNF0_Fn106(INT8U *pBuf)
{
	INT16U i;
	INT16U pNum;
    INT8U tUseNum;
	INT8U n;

	for(i=0; i<MAX_DAU; i++)
	{
		if(mDAU[i].aDAddr[0] == pBuf[0]
		&& mDAU[i].aDAddr[1] == pBuf[1]
		&& mDAU[i].aDAddr[2] == pBuf[2]
		&& mDAU[i].aDAddr[3] == pBuf[3]
		&& mDAU[i].aDAddr[4] == pBuf[4]
		&& mDAU[i].aDAddr[5] == pBuf[5])
		{
			pNum = i;    // 定位节点序号
			break;
		}
	}

	pBuf[0] = mDAU[i].aDAddr[0];
	pBuf[1] = mDAU[i].aDAddr[1];
	pBuf[2] = mDAU[i].aDAddr[2];
	pBuf[3] = mDAU[i].aDAddr[3];
	pBuf[4] = mDAU[i].aDAddr[4];
	pBuf[5] = mDAU[i].aDAddr[5];

	if(pBuf[6] == 0)
	{
		for(n=0; n<5; n++)
		{
			mth_ReadmMath(&(pBuf[7+n*39]), pNum, n, &tUseNum);
		}
        pBuf[202] = tUseNum;
		uart_Answer_Data(203);
	}
	else if(pBuf[6] == 1)
	{
		for(n=0; n<5; n++)
		{
			mth_ReadmMath(&(pBuf[7+n*39]), pNum, 5+n, &tUseNum);
		}
        pBuf[202] = tUseNum;
		uart_Answer_Data(203);
	}
	else if(pBuf[6] == 2)
	{
		for(n=0; n<3; n++)
		{
			mth_ReadmMath(&(pBuf[7+n*39]), pNum, 10+n, &tUseNum);
		}
        pBuf[124] = tUseNum;
		uart_Answer_Data(125);
	}
}

// 透传数据到掌机
void DL2013_AFNF0_Fn201(INT8U *pBuf)  // 掌机到集中器使用F0H 200
{
    INT16U i;
    
    gDtHand *tHand=(gDtHand*)(mRf.aBuf);
    mRf.i1Com   = RF_DEV_HAND;
    mRf.i1Layer = 1;
    
    tHand->i1Com=0x34;
    tHand->i1Len=pBuf[0];
    for(i=0; i<tHand->i1Len; i++)
    {
        tHand->aBuf[i] = pBuf[1+i];
    }

    if(dl_RfStruct(mRf.BufTx,&mRf)==TRUE1)
    {
        drv_RfSend(mRf.BufTx,CH_TYPE_HAND);
    }
    uart_Answer_Yes();
}
#endif
