//================= 32W电机的参数设置   ========================================
#define RatedC 1.5   // 额定电流 4.0 FOR 64w 电机
#define RatedV 24    // 额定电压
#define EncoderCPR 1000  //编码器的分辨率
#define PolePairs 4      //极对数
#define BaseSpeed 3000   //电机额定转速
#define UMAX 1.5         //外环控制信号的饱和限幅值，与额定电流 对应
#define OutLoopTS 0.002  //外环采样周期

#define E_Ding_DianLiu (1.414*RatedC)   //电机的额定电流的有效值  单位A
#define GuoliuZhi (15*RatedC)     // 用于过流保护， 放大了10倍
#define MaxDCV (1.25*RatedV)      // 最大瞬时电压

#define MaxPulses (4*EncoderCPR)    //每圈的QEP脉冲数
#define SectorWidth (MaxPulses/PolePairs/6)  //转子初始定位的扇区脉冲数，BuChang=166
#define PosRevScale (1.0/MaxPulses)     //QEP脉冲数 折算为 圈数的比例系数
#define SpeedScale (PosRevScale/OutLoopTS/50)  // speed = pu value
#define SpeedRpsScale (PosRevScale*37500000)  // speed=scale/t2_t1 = rps value


