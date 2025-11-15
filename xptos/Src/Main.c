//*****************************************************************************************************
//Flash和RAM软件版本切换说明(程序默认为ram版本)
//
//一.切换为Flash烧写版本方法
//1.将主程序中的:MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//               InitFlash();
//  两个函数取消注释
//2.将工程中的28335_RAM_lnk.cmd从工程中删除，添加CMD文件夹下的F28335.cmd文件，全编译一次即可烧写。
//
//二.切换为RAM在线仿真版本方法
//1.将主程序中的:MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//               InitFlash();
//  两个函数注释掉
//2.将工程中的F28335.cmd从工程中删除，添加CMD文件夹下的28335_RAM_lnk.cmd文件，全编译一次即可。
//
//*****************************************************************************************************
#include "vptos.h"
//#include "ptos.h"
#include <math.h>
#include <dlog4ch.h>
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
// Prototype statements for functions found within this file.
int T_1=0;
#define pi 3.1415926
#define FALSE 0
#define TRUE  1

interrupt void EPWM_1_INT(void);                //转子初始化，控制算法，SVPWM实现
interrupt void INT3_ISR(void);                  // 声明中断函数，清除12组应答位
void Init_SiShu(void);                          //初始化电机保护参数
void performance_metrics_update(float y);
//RCNS pos_ctrl = RCNS_DEFAULTS;
PTOC pos_ctrl = PTOC_DEFAULTS;                  //声明结构体PTOC类型变量，并初始化

DLOG_4CH dlog = DLOG_4CH_DEFAULTS;              //声明结构体DLOG类型变量图像，显示指针函数


//*****************************************************************************************************
//全局变量定义与初始化
//***************************************************************************************************** 

float32 i=0;
float32 j=0;
float32 k=0;
Uint16 T1Period=0;     // T1定时器周期(Q0)
Uint16 T3Period = 0;
float32 Modulation=0.25;    // 调制比
int16 MPeriod=0;
int32 Tmp=0;
_iq PolePairs=_IQ(4); 

//:::::::::::::::::::::::::::位置环变量定义:::::::::::::::::::::::::::
long Place_now=0;//位置变量值定义
Uint16 PlaceSetBit=0;  //位置设定标志位
Uint16 PosEnable=1;//位置控制 使能  1 使能 ;  0 -> 调速
int32 PosCount = 0;
float32 pscale=2;

//===============转子初始位置定位=============================  
Uint16 LocationFlag=TRUE;
Uint16 LocationEnd=0;
Uint16 Position=1;
Uint16 PositionPhase60=1;
Uint16 PositionPhase120=2;
Uint16 PositionPhase180=3;
Uint16 PositionPhase240=4;
Uint16 PositionPhase300=5;
Uint16 PositionPhase360=6;

//===============DAC模拟===================================== 
_iq DACTemp0=0;
_iq DACTemp1=0;
_iq DACTemp2=0; 

_iq MfuncC1=0;
_iq MfuncC2=0;
_iq MfuncC3=0; 
Uint16 ZhengFan=1;  
 
//===============转子速度计算===================================== 
Uint16 SpeedLoopPrescaler = 20;     // 速度环定标
Uint16 SpeedLoopCount = 1;          // 速度环计数
_iq NewRawTheta=0;
_iq OldRawTheta=0;
_iq SpeedRpm=0;                     //速度，单位：转/每分钟
Uint16 Hall_Fault=0;
_iq RawThetaTmp=0;
float32 SpeedRef=0;
_iq Speed=0;                        //速度，标幺值

//===============转子角度计算===================================
Uint16 DirectionQep=0;               //转子旋转方向
_iq RawTheta=0;
_iq OldRawThetaPos = 0;

_iq TotalPulse=0; 

_iq MechTheta = 0;                   //机械角度，单位：度
_iq ElecTheta = 0;                   //电气角度，单位：度
_iq AnglePU=0;                       //角度标幺化
_iq Cosine=0;
_iq Sine=0;


//===============控制绕组电流计算============================ 
_iq ia=0;
_iq ib=0;
_iq ic=0;
_iq ialfa=0;
_iq ibeta=0; 
_iq id=0;
_iq iq=0; 

//===============PI控制器参数计算============================ 
_iq ID_Given=0;
_iq ID_Ref=0;
_iq ID_Fdb=0;
_iq ID_Error=0;  

_iq ID_Up=0;
_iq ID_Up1=0;
_iq ID_Ui=0;
_iq ID_OutPreSat=0;
_iq ID_SatError=0;
_iq ID_OutMax=_IQ(1);
_iq ID_OutMin=_IQ(-1); 
_iq ID_Out=0;

_iq IQ_Given=0; _iq IQ_Given_test=1200;
_iq IQ_Ref=0;
_iq IQ_Fdb=0;
_iq IQ_Error=0; 
 
_iq IQ_Up=0;
_iq IQ_Up1=0;
_iq IQ_Ui=0;
_iq IQ_OutPreSat=0;
_iq IQ_SatError=0;
_iq IQ_OutMax=_IQ(1);
_iq IQ_OutMin=_IQ(-1); 
_iq IQ_Out=0; 

_iq Speed_Given=_IQ(0.2); //速度给定    标幺值 0.2==>600RPM，最高转速1.0==>3000RPM
_iq Speed_Ref=0;
_iq Speed_Fdb=0;
_iq Speed_Error=0;


_iq Speed_Up=0;
_iq Speed_Up1=0;
_iq Speed_Ui=0;
_iq Speed_OutPreSat=0;
_iq Speed_SatError=0;
_iq Speed_OutMax=_IQ(0.99999);
_iq Speed_OutMin=-_IQ(0.99999);
_iq Speed_Out=0;
Uint16 Speed_run=0;

//===============SVPWM计算==================================== 
Uint16 Sector = 0; 
_iq Ualfa=0;
_iq Ubeta=0;
_iq Ud=0;
_iq Uq=0;
_iq B0=0;
_iq B1=0;
_iq B2=0;
_iq X=0;
_iq Y=0;
_iq Z=0;
_iq t1=0;
_iq t2=0;
_iq Ta=0;
_iq Tb=0;
_iq Tc=0;
_iq MfuncD1=0;
_iq MfuncD2=0;
_iq MfuncD3=0;
//===================================================================
Uint16 Run_PMSM=2;
float32 TEMP2=0;
_iq MechScaler=_IQ(0.0);
_iq SpeedScaler=_IQ(0.00);
Uint16 speed_give=0;
Uint16 HallAngle=0;
Uint16 BuChang=166;
int16 TotalCnt=4000;
_iq RawCnt1=0;
_iq RawCnt2=0;
Uint16 ShangDian_Err=0;


//========================速度环PI参数=================================
_iq Speed_Kp=_IQ(2.5);
_iq Speed_Ki=_IQ(0.007);
_iq Speed_Kd=_IQ(0);
//=====================================================================

//========================Q轴电流环PI参数==============================
_iq IQ_Kp=_IQ(0.1221);
_iq IQ_Ki=_IQ(0.061);
//=====================================================================

//========================D轴电流环PI参数==============================
_iq ID_Kp=_IQ(0.1221);
_iq ID_Ki=_IQ(0.061);
//=====================================================================

long PlaceSet=10000;//位置环脉冲数
Uint16 PlaceEnable=0;//位置环使能  1 使能 ;  0 禁止

//=====================参数设置========================================
float32 E_Ding_DianLiu=4.0;        //设置电机的额定电流的有效值  单位A
Uint16 BaseSpeed=3000;              //设置电机额定转速

                
/*****图像显示****************/
int16 DlogCh1=0;
int16 DlogCh2=0;
int16 DlogCh3=0;
int16 DlogCh4=0;

/*****转子初始化****************/
Uint16 LockRotorNum = 0;
float p_cycles_now=0;
float p_cycles_old=0;
float p_Speed=0;

float p_cycles_now1=0;
float p_cycles_old1=0;
float p_Speed1=0;

float32 yk=0;

_iq MaxOut=_IQ(0.99999);  //0.99999
_iq MinOut=_IQ(-0.99999);

float32 UMAX=4.0;
float32 ctrl_uk=0;
float32 OutLoopCnt=0;
float32 PosRefScaler=500;
float32 PosRef=1.0;

//===============performance=============================
float32 overshoot = 0.0f;
float32 steady_state_error = 0.0f;
float32 rise_time = 0.0f;
float32 y_max = 0.0f;
float32 ref = 0;
int sample_count = 0;
int rise_flag = 0;
float32 sample_period = 0.02;   // 采样周期，秒

void main(void)
 {

    InitSysCtrl();

    InitGpio();
    Pwm_EN_1;

    DINT;

    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
 
   InitPieVectTable();
 
   EALLOW;  // This is needed to write to EALLOW protected registers 

   PieVectTable.EPWM1_INT=&EPWM_1_INT;
   PieVectTable.XINT3=&INT3_ISR;

   EDIS;    // This is needed to disable write to EALLOW protected registers
 
   // InitCpuTimers();
   InitSci_C();
   InitSci_B();
   InitSpi();


   //MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
   //InitFlash();
  
   InitEPwm_1_2_3();        //pwm初始化
   QEP_Init();              //qep初始化

   Init_SiShu();
   ADC_Soc_Init();
   
 
   eva_close();             //复位关键电机控制变量
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   

   if(AD_BUF[7]<150)
   {
       Pwm_EN_0;//允许PWM使能

   }
   else
   {
       Pwm_EN_1;//禁止PWM使能
       ShangDian_Err=1;

   }
    
   // 初始化数据记录模块   Initialize DATALOG module
    dlog.iptr1 = &DlogCh1;
    dlog.iptr2 = &DlogCh2;
    dlog.iptr3 = &DlogCh3;
    dlog.iptr4 = &DlogCh4;
    dlog.trig_value = 1 ;   //CCS每次图像移动的个数  0x1
    dlog.size = 0x400;     //数据的长度，一次最多采集 1024 个采样点
    dlog.prescalar=3;      //CCS图形采样频率=1/prescalar/(1ms)（主中断周期）=1000Hz
    dlog.init(&dlog);


//    //eptos控制律的初始化
//    pos_ctrl.umax=UMAX;
//    pos_ctrl.Ts=0.002;
//    pos_ctrl.b=500;  // b越大，超调越小，运行越平稳     //模型参数
//    pos_ctrl.a=-3;  // a<=0;
//    pos_ctrl.zeta=0.85;  //线性控制的阻尼系数
//    pos_ctrl.omega=60;    //线性控制的自然频率
//    pos_ctrl.zeta0=0.8;  //观测器极点的阻尼系数 0.7071
//    pos_ctrl.omega0=200;    //观测器极点的自然频率
//    pos_ctrl.fd=0.5;    //扰动补偿的稳态系数
//    pos_ctrl.alpha=0.95;    //非线性函数的参数
//    pos_ctrl.init(&pos_ctrl);

    //PTOS控制律的初始化
    pos_ctrl.umax=UMAX;
    pos_ctrl.Ts=0.002;   // 0.001 or 0.002
    //      pos_ctrl.a=-3;
    pos_ctrl.b=500;  //模型参数
    pos_ctrl.a=-3;
    pos_ctrl.alpha=0.95;  //
    pos_ctrl.zeta=0.85;  //  阻尼系数
    pos_ctrl.omega=60;//80;    //自然频率
    pos_ctrl.zeta0=0.80;  //观测器极点的阻尼系数
    pos_ctrl.omega0=120;//62;    //观测器极点的自然频率
    pos_ctrl.gama=0.85;//
    pos_ctrl.lambda=3;
    pos_ctrl.fd=0.3;    //扰动补偿的稳态系数
    pos_ctrl.init(&pos_ctrl);   //调用初始化函数


   DELAY_US(1000000);
    
   
   IER |= M_INT3;
   IER |= M_INT9;
   IER |= M_INT12;

   //中断使能
   //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//timer0
   PieCtrlRegs.PIEIER3.bit.INTx1=1;//epwm1int
   PieCtrlRegs.PIEIER9.bit.INTx3=1;//scib
   PieCtrlRegs.PIEIER12.bit.INTx1=1;//xint3
    
   Init_lcd();//初始化显示屏
 
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

       //SpeedRef = 0.1;

   for(;;)
     {
        //CPU_RUN();
        DC_Link();
        deal_key();//对中断读到的按键进行判断，并进行一系列的操作
        LCD_DIS();//轮流在 LCD 上显示不同信息
        //TX232_ChuLi();

     }

}

interrupt void EPWM_1_INT(void)
{
       _iq t_01,t_02;
       //IPM_BaoHu();
    
   
  Read_key();//获取按键的信息------判断哪个按键按下
  Ad_CaiJi(); //采集电机的三相电流ia,ib,ic;用于后面Clark,Park变化，进一步得到iq，id
  //JiSuan_Dl();//对采集到的电流进行计算，判断是否启动过流保护


if(Run_PMSM==1&&IPM_Fault==0)
{

     if(LocationFlag == TRUE)    //代表转子要开始初始化
      {
          if(LockRotorNum < 25000)//35000
          {
              LockRotorNum ++;
              if(LockRotorNum < 9500)       //25000定位转子
              {
                  //分别设置三相PWM的比较寄存器CMPA的值--------设置三相PWM输出的占空比产生空间电压矢量，使转子到达指定位置
                  EPwm1Regs.CMPA.half.CMPA = 1321;
                  EPwm2Regs.CMPA.half.CMPA = 652;
                  EPwm3Regs.CMPA.half.CMPA = 652;
                  /*EPwm1Regs.CMPA.half.CMPA = 2337;
                  EPwm2Regs.CMPA.half.CMPA = 1412;
                  EPwm3Regs.CMPA.half.CMPA = 1412;*/

              }
              else                          //稳住转子、防止扰动
              {
                  EPwm1Regs.CMPA.half.CMPA = 3375;
                  EPwm2Regs.CMPA.half.CMPA = 3375;
                  EPwm3Regs.CMPA.half.CMPA = 3375;
              }
          }
          else
          {
              EQep1Regs.QPOSCNT = 0;//清零编码器位置
              LocationFlag = FALSE; //LocationFlag = FALSE 代表转子初始化完毕，开始闭环控制
              Modulation=0.95;//限制 PWM 输出不超过 95%
          }

      }

        

//=====================================================================================================
//初始位置定位结束，开始闭环控制
//=====================================================================================================
    if(LocationFlag == FALSE)
    {
//======================================================================================================
//QEP角度计算
//====================================================================================================== 

// 旋转方向判定 
        DirectionQep = EQep1Regs.QEPSTS.bit.QDF;//QDF（Direction Flag）位，表示转子的旋转方向

        RawTheta = _IQ(EQep1Regs.QPOSCNT);//取QEP编码器的原始计数值

        if(DirectionQep ==1) //递增计数，代表顺时针；
        {
            if((OldRawThetaPos>_IQ(3900)) && (RawTheta<_IQ(900)))   //处理编码器计数从接近最大值（比如3900附近）回绕到小值（小于900）时，表示转子转过了一圈。
            {
                PosCount += TotalCnt;
            }
            Place_now= _IQtoF(RawTheta)+PosCount;
            OldRawThetaPos = RawTheta;
        }

        else if(DirectionQep ==0)//递减计数，代表逆时针
        {
            if((RawTheta>_IQ(3000)) && (OldRawThetaPos<_IQ(1000)))
            {
                PosCount -= TotalCnt;
            }
            Place_now = _IQtoF(RawTheta)+PosCount;
            OldRawThetaPos = RawTheta;
        }

        MechTheta = _IQmpy(2949,RawTheta);//这里2949是Q15格式，转化为浮点型即为：2949/32768=0.09；一个QCLK代表的角度为：360/4000=0.09

        //以下为防止机械角度超过360度或者负360度
        if(MechTheta>_IQ(360))
        {MechTheta=MechTheta-_IQ(360);}
        if(MechTheta<_IQ(-360))
        {MechTheta=MechTheta+_IQ(360);}
        //以上截止

        //由机械角度得出电角度,注：电角度是可以大于360，因为IQ15只能表示-1-1之间，所以这样的运算可以自动取小数部分，也就是使得电角度小于360. 这用于后面park等变化
        ElecTheta = _IQmpy(PolePairs,MechTheta);


        AnglePU=_IQdiv(ElecTheta,_IQ(360)); //电角度的标幺化
        Sine = _IQsinPU(AnglePU);           // 计算电角度对应的正弦值
        Cosine = _IQcosPU(AnglePU);         // 计算电角度对应的余弦值


//======================================================================================================
//QEP速度计算
//====================================================================================================== 

        if (SpeedLoopCount>=SpeedLoopPrescaler)//判断是否达到执行位置控制计算的时间点
        {
            OldRawTheta = NewRawTheta;//保存上一周期的编码器位置
            SpeedLoopCount=1;
            RawThetaTmp=0;

//=================位置控制(指数滑模面以及超螺旋控制)===================================

            //位置控制参考 pos_ctrl.Ref 在 0 和 PosRef 之间周期切换
            OutLoopCnt++;

            if (OutLoopCnt>=PosRefScaler)
            {
                OutLoopCnt=0 ;

                if (pos_ctrl.Ref == 0)
                    pos_ctrl.Ref = PosRef ;
                else
                    pos_ctrl.Ref = 0;

                pscale=fabs(2.0*PosRef);
                if (pscale<0.5)
                    pscale=1;
            }

            //把编码器的脉冲计数（位置值）转换成机械角度或者标幺位置反馈
             yk= Place_now*0.00025;//(Place_now*0.09)/360 = 0.00025;
                           //rk = v_Ref * 2 * pi;
                           //c_count = sin( 5 * v_tk);
                           //rk = pi * c_count;



             //pos_ctrl.vk=p_Speed/60;
             pos_ctrl.Fdb = yk;             // 设置位置反馈值，即当前转子的位置
             pos_ctrl.calc(&pos_ctrl);      // 调用位置控制器计算函数，根据参考和反馈计算控制输出
             ctrl_uk = pos_ctrl.Out;        // UMAX;获取位置控制器的输出值

             IQ_Given =_IQdiv(_IQ(ctrl_uk),_IQ(UMAX));  //位置控制器的输出信号归一化
             if (IQ_Given>MaxOut)
                 IQ_Given=MaxOut;
             else if(IQ_Given<MinOut)
                 IQ_Given=MinOut;

             performance_metrics_update(yk);

//=================显示模块===================================

            Speed_run=1;


//          DlogCh1=(int16)_IQ15(pos_ctrl.vk/100);
//          DlogCh2=(int16)_IQ15(yk/PosRef/2);
//          DlogCh3=(int16)_IQ15(pos_ctrl.dk/4);
//          DlogCh4=(int16)(IQ_Given);
//            dlog.update(&dlog);

            DlogCh1=(int16)IQ_Given;  //_IQ15(ctrl_uk)
            DlogCh2=(int16)_IQ15(pos_ctrl.Fdb/pscale);
            DlogCh3=(int16)_IQ15(pos_ctrl.vk/50);
            DlogCh4=(int16)_IQ15(pos_ctrl.dk/UMAX);
            dlog.update(&dlog);

        }
        else
        {
            SpeedLoopCount++;
        }


        //Clarke变换 + Park变换 组合的简化形式
        ialfa=ia;
        ibeta=_IQmpy(ia,_IQ(0.57735026918963))+_IQmpy(ib,_IQ(1.15470053837926));

        id = _IQmpy(ialfa,Cosine) +_IQmpy(ibeta,Sine);
        iq = _IQmpy(ibeta,Cosine)- _IQmpy(ialfa,Sine);


//======================================================================================================
//IQ电流PID调节控制
//======================================================================================================
        IQ_Ref=IQ_Given;
        IQ_Fdb=iq;

        IQ_Error=IQ_Ref-IQ_Fdb;

        IQ_Up=_IQmpy(IQ_Kp,IQ_Error);           //比例控制量
        IQ_Ui=IQ_Ui + _IQmpy(IQ_Ki,IQ_Up);      //对比例项积分

        IQ_OutPreSat=IQ_Up+IQ_Ui;

        if(IQ_OutPreSat>IQ_OutMax)
            IQ_Out=IQ_OutMax;
        else if(IQ_OutPreSat<IQ_OutMin)
            IQ_Out=IQ_OutMin;
        else
            IQ_Out=IQ_OutPreSat;


        Uq=IQ_Out;

//======================================================================================================
//ID电流PID调节控制
//======================================================================================================  
        ID_Ref=ID_Given;
        ID_Fdb=id;

        ID_Error=ID_Ref-ID_Fdb;

        ID_Up=_IQmpy(ID_Kp,ID_Error);
        ID_Ui=ID_Ui+_IQmpy(ID_Ki,ID_Up);

        ID_OutPreSat=ID_Up+ID_Ui;

        if(ID_OutPreSat>ID_OutMax)
            ID_Out=ID_OutMax;
        else if(ID_OutPreSat<ID_OutMin)
            ID_Out=ID_OutMin;
        else
            ID_Out=ID_OutPreSat;

        Ud=ID_Out;


//======================================================================================================
//IPark变换
//====================================================================================================== 
        Ualfa = _IQmpy(Ud,Cosine) - _IQmpy(Uq,Sine);
        Ubeta = _IQmpy(Uq,Cosine) + _IQmpy(Ud,Sine);

//======================================================================================================
//SVPWM实现
//====================================================================================================== 
        //将两相静止坐标系（α-β）中的电压向量 Ualfa, Ubeta 投影到三相坐标系 B0、B1、B2 上
        B0=Ubeta;
        B1=_IQmpy(_IQ(0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta);// 0.8660254 = sqrt(3)/2
        B2=_IQmpy(_IQ(-0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2

        //根据三相坐标系正负判断扇区
        Sector=0;
        if(B0>_IQ(0)) Sector =1;
        if(B1>_IQ(0)) Sector =Sector +2;
        if(B2>_IQ(0)) Sector =Sector +4;

        //用于计算扇区内占空比
        X=Ubeta;//va
        Y=_IQmpy(_IQ(0.8660254),Ualfa)+ _IQmpy(_IQ(0.5),Ubeta);// 0.8660254 = sqrt(3)/2 vb
        Z=_IQmpy(_IQ(-0.8660254),Ualfa)+ _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2 vc


            if(Sector==1)
            {
                t_01=Z;
                t_02=Y;

                if((t_01+t_02)>_IQ(1))
                {
                    t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
                    t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));
                }
                else
                {
                    t1=t_01;
                    t2=t_02;
                }

                //t0/2 + t1 + t2 + t0/2 = T
                Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
                Ta=Tb+t1;
                Tc=Ta+t2;
            }
        else if(Sector==2)
        {
            t_01=Y;
            t_02=-X;

            if((t_01+t_02)>_IQ(1))
            {
                t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
                t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));
            }
            else
            {
                t1=t_01;
                t2=t_02;
            }

            Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Tc=Ta+t1;
            Tb=Tc+t2;
        }
        else if(Sector==3)
        {
            t_01=-Z;
            t_02=X;

            if((t_01+t_02)>_IQ(1))
            {
                t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
                t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));
            }
            else
            {
                t1=t_01;
                t2=t_02;
            }

            Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Tb=Ta+t1;
            Tc=Tb+t2;
        }
        else if(Sector==4)
        {
            t_01=-X;
            t_02=Z;

            if((t_01+t_02)>_IQ(1))
            {
                t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
                t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));
            }
            else
            {
                t1=t_01;
                t2=t_02;
            }

            Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Tb=Tc+t1;
            Ta=Tb+t2;
        }
        else if(Sector==5)
        {
            t_01=X;
            t_02=-Y;

            if((t_01+t_02)>_IQ(1))
            {
                t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
                t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));
            }
            else
            {
                t1=t_01;
                t2=t_02;
            }

            Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Tc=Tb+t1;
            Ta=Tc+t2;
        }
        else if(Sector==6)
        {
            t_01=-Y;
            t_02=-Z;

            if((t_01+t_02)>_IQ(1))
            {
                t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
                t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));
            }
            else
            {
                t1=t_01;
                t2=t_02;
            }

            Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
            Ta=Tc+t1;
            Tb=Ta+t2;
        }

        // 将三相 PWM 的占空比起始时间转换为中心对齐 PWM 比较器输入----[-1,1]
        MfuncD1=_IQmpy(_IQ(2),(_IQ(0.5)-Ta));
        MfuncD2=_IQmpy(_IQ(2),(_IQ(0.5)-Tb));
        MfuncD3=_IQmpy(_IQ(2),(_IQ(0.5)-Tc));

//======================================================================================================
//EVA全比较器参数赋值，用于驱动电机
//====================================================================================================== 
    MPeriod = (int16)(T1Period * Modulation);              // Q0 = (Q0 * Q0)

    Tmp = (int32)MPeriod * (int32)MfuncD1;                    // Q15 = Q0*Q15，计算全比较器CMPR1赋值
     EPwm1Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    Tmp = (int32)MPeriod * (int32)MfuncD2;                    // Q15 = Q0*Q15，计算全比较器CMPR2赋值
     EPwm2Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    Tmp = (int32)MPeriod * (int32)MfuncD3;                    // Q15 = Q0*Q15，计算全比较器CMPR3赋值
     EPwm3Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

         
    }
    }


if(DC_ON_flag==1)
{

        if(U_dc_dis<10)//执行停机命令
        {
        eva_close();
        Run_PMSM=2;
        DC_ON_flag=0;

        }

}


EPwm1Regs.ETCLR.bit.INT=1;//清除中断标志位
PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}



/*interrupt void SCIBRX_ISR(void)     // SCI-B
{

    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}*/

void Init_SiShu(void)
{ 

 GuoliuZhi=15*E_Ding_DianLiu;
 E_Ding_DianLiu=1.414*E_Ding_DianLiu;
 }

interrupt void INT3_ISR(void)
{ 
   
PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

void performance_metrics_update(float y)
{
    sample_count++;
    ref = PosRef;

    if(y > y_max)
    {
        y_max = y;
    }

    // 计算超调量（以相对值表示，负值算0）
    float os = (y_max - ref) / ref;
    if(os < 0) os = 0;
    overshoot = os;
    y_max = 0;
//    // 稳态误差（绝对值）
//    steady_state_error = fabsf(y - ref);

    // 记录上升时间，首次超过0.9*ref
    if(!rise_flag && y >= 0.9f * ref && y>1e-6)
    {
        rise_time = sample_count * sample_period;
        rise_flag = 1;
    }


}

//===========================================================================
// No more.
//===========================================================================
