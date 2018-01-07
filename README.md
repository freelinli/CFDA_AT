# CFDA_AT



----------------------------------------------------------------------------------------------------------------------------------------
2018/1/7
指令增加


// 实现从节点点名 04H F2

AT+NRC

OK

// 查主节点状态字及通信速率 03H  F5

AT+SCCR?

1,0,4,31,0,0,0,0  

AT+SCCR?
 
1,0,1,31,0  


// 发送测试 04H  F1 注意等待时间不要过长


AT+TSST=100

OK

AT+TSST=5

OK

AT+TSST=300

ERROR:87

// 透传数据到掌机  AT+TTTH


AT+TTTH=5,abcde7

ERROR

AT+TTTH=513,abcde

ERROR:88




----------------------------------------------------------------------------------------------------------------------------------------
2018/1/7
第一次版本上传与完善，实现功能接口框架及以下功能



// 01H F1 硬件初始化 

AT+HINT
OK

// 05H F1 设置主节点地址

AT+HNA=123456789012
OK

// 03H F4 查询主节点地址
AT+HNA?
123456789012

0D 31 12 33 34 35 36 37 38 39 30 31 32 0D 


// 03H F1 查询版本信息  
AT+MVINFO?
CFMJ,170816,80.03,CFMJ,170816,2.0

// 03H F100 查询场强门限

AT+RSSIT?

96


// 05H F100 设置场强门限
AT+RSSIT=100

OK

AT+RSSIT=50

OK


AT+RSSIT=140

ERROR:81


AT+RSSIT=97

OK

AT+RSSIT=?
Invalid command

// 读取从节点数量 10H F1

AT+NNUM?

4,601
// 参数区初始化 01 F2 01H F3
AT+PINT

OK


// 读取内部版本号 F0H F101
AT+RIV?

17,128.3,36

//查询真实频道号 F0H F102
AT+CACRC?

9


//查询网络规模 10H F100
AT+SSCL?

4


//设置网络规模 11H F100
AT+SSCL=14

OK

// 设置中心节点时间 05H F101

AT+RTC=163330

OK


// 查询中心节点时间 03H F101

AT+RTC?

163459


//读取从节点监控最大超时时间

AT+NMMT?

60


//设置从节点监控最大超时时间

AT+NMMT=20

OK


AT+NMMT=2000

ERROR:84


AT+NMMT=256

ERROR:84

// 启动维护 11H F101

AT+RTMT

OK


//启动组网  11H F102


AT+RTRS

OK


//路由请求集中器时钟  14H F2

AT+GCLK=131212

OK

AT+RTC?

AT+GCLK=131214




// 查询无线通信参数 03H F8

AT+HNCP?

1,0




// 设置无线通信参数 05H F5

AT+HNCP=2,3

OK

AT+HNCP=10,3

OK


AT+HNCP=10,2

OK

AT+HNCP=13,5

ERROR:86

ERROR:85

// 允许/禁止从节点上报 05H F2
AT+NEREP=1

OK

AT+NEREP=0

OK

AT+NEREP=2
 
ERROR
