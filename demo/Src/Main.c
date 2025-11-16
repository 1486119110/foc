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

#include  <math.h>
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "motor.h"    // define the parameters of motor

//#include "rcns.h"
//#include "ptos.h"
#include "vptos.h"
PTOC pos_ctrl = PTOC_DEFAULTS;
//RCNS pos_ctrl = RCNS_DEFAULTS;

#include "dlog4ch.h"
DLOG_4CH dlog = DLOG_4CH_DEFAULTS; //图像显示指针函数
/*****图像显示****************/
int16 DlogCh1=0;
int16 DlogCh2=0;
int16 DlogCh3=0;
int16 DlogCh4=0;

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void); 
interrupt void EPWM_1_INT(void);
interrupt void SCIBRX_ISR(void);
interrupt void INT3_ISR(void);
void OutLoop_Control(void);
void performance_metrics_update(float y);
//void Init_SiShu(void);

//*****************************************************************************************************
//全局变量定义与初始化
//***************************************************************************************************** 
float32 i=0;
float32 j=0;
float32 k=0;
Uint16 IsrTicker = 0;
Uint16 BackTicker = 0; //用于次数计数
Uint16 T1Period=0;     // T1定时器周期(Q0)
Uint16 T3Period = 0;   
float32 Modulation=0.25;    // 调制比
int16 MPeriod=0;
int32 Tmp=0;


float32 MfuncF1=0;
float32 MfuncF2=0;
float32 MfuncF3=0;  
//===============转子初始位置定位=============================  
Uint16 LockRotorNum = 0;
Uint16 LocationFlag=1;
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
Uint16 SpeedRefScaler = 500;     // 速度环给定的变化周期
Uint16 SpeedLoopCount = 1;          // 速度环计数  
float32 SpeedRef=0;
float32 SpdRef=0;
_iq Speed=0;                        //速度，标幺值
_iq SpeedRpm=0;                     //速度，单位：转/每分钟
Uint16 Hall_Fault=0;
float32 Speed_rps=0;
unsigned int t2_t1;

//===============转子角度计算===================================
Uint16 DirectionQep=0;               //转子旋转方向
Uint32 RawTheta=0;
Uint32 OldRawTheta=0;
float32 RawThetaTmp=0;

float32 MechTheta = 0;             //机械角度，单位：度-》Rev
float32 ElecTheta = 0;             //电气角度，单位：度-》Rev
_iq	AnglePU=0;                     //角度标幺化
_iq	Cosine=0;
_iq	Sine=0;


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

_iq IQ_Given=0;
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
_iq MaxOut=_IQ(0.99999);  //0.99999
_iq MinOut=_IQ(-0.99999);
_iq Speed_Out=0;  

//===============SVPWM计算==================================== 
Uint16 Sector = 0; 
_iq	Ualfa=0;  		
_iq	Ubeta=0;		
_iq	Ud=0;		
_iq	Uq=0;			
_iq	B0=0;			
_iq	B1=0;
_iq	B2=0;
_iq	X=0;
_iq	Y=0;
_iq	Z=0;
_iq	t1=0;
_iq	t2=0;
_iq	Ta=0;
_iq	Tb=0;
_iq	Tc=0;
_iq	MfuncD1=0;
_iq	MfuncD2=0;
_iq	MfuncD3=0; 
//===================================================================
Uint16 Run_PMSM=2;
float32 TEMP2=0;

Uint16 speed_give=0;
Uint16 HallAngle=0;


Uint16 ShangDian_Err=0;

// for 64W MOTOR
//========================速度环PI参数=================================
_iq Speed_Kp=_IQ(2.5);
_iq Speed_Ki=_IQ(0.007);
//=====================================================================

//========================Q轴电流环PI参数==============================
_iq IQ_Kp=_IQ(0.1221);
_iq IQ_Ki=_IQ(0.061);
//=====================================================================

//========================D轴电流环PI参数==============================
_iq ID_Kp=_IQ(0.1221);
_iq ID_Ki=_IQ(0.061);
//=====================================================================

// for 32W MOTOR
////========================速度环PI参数=================================
//_iq Speed_Kp=_IQ(8);
//_iq Speed_Ki=_IQ(0.005);
////=====================================================================
//
////========================Q轴电流环PI参数==============================
//_iq IQ_Kp=_IQ(0.3);
//_iq IQ_Ki=_IQ(0.002);
////=====================================================================
//
////========================D轴电流环PI参数==============================
//_iq ID_Kp=_IQ(0.3);
//_iq ID_Ki=_IQ(0.002);
////=====================================================================

//:::::::::::::::::::::::::::位置环变量定义:::::::::::::::::::::::::::
Uint16 PosEnable=1;//位置控制 使能  1 使能 ;  0 -> 调速
int32 PosRevCnt = 0; // in Rev
long Place_now = 0;   // 绝对脉冲计数
float32 PosInRev = 0;
float32 PosInit=0;
float32 PosRef=1; //In Rev
float32 PosErr=0;
float32 pscale=2;
float32 yk=0;
int LoopCnt=0;
Uint16 OutLoopScaler=20;
int OutLoopCnt=0;
Uint16 PosRefScaler=500; //600

float32 Speed_pu;
float32 speed_pu_m;
float32 speed_pu_t;

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
  // PieVectTable.TINT0 = &cpu_timer0_isr; 
   PieVectTable.EPWM1_INT=&EPWM_1_INT;
   PieVectTable.SCIRXINTB= &SCIBRX_ISR;   //设置串口B接受中断的中断向量
   PieVectTable.XINT3=&INT3_ISR;

   EDIS;    // This is needed to disable write to EALLOW protected registers
 

   InitSci_C();
   InitSci_B();
   InitSpi();

   
/*
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
   InitFlash();
*/

   InitEPwm_1_2_3();//pwm初始化
   QEP_Init(); //qep初始化

//   Init_SiShu();
   ADC_Soc_Init();
   
 
   eva_close();
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
    
    
    DELAY_US(1000000);
    
   
   IER |= M_INT3;
   IER |= M_INT9;
   IER |= M_INT12;
   //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//timer0
   PieCtrlRegs.PIEIER3.bit.INTx1=1;//epwm1int
   PieCtrlRegs.PIEIER9.bit.INTx3=1;//scib
   PieCtrlRegs.PIEIER12.bit.INTx1=1;//xint3
    
   Init_lcd();
 
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM
    
//   //eptos控制律的初始化
//   pos_ctrl.umax=UMAX;
//   pos_ctrl.Ts=0.002;
//   pos_ctrl.b=500;  // b越大，超调越小，运行越平稳     //模型参数
//   pos_ctrl.a=-3;  // a<=0;
//   pos_ctrl.zeta=0.85;  //线性控制的阻尼系数
//   pos_ctrl.omega=60;    //线性控制的自然频率
//   pos_ctrl.zeta0=0.8;  //观测器极点的阻尼系数 0.7071
//   pos_ctrl.omega0=200;    //观测器极点的自然频率
//   pos_ctrl.fd=0.5;    //扰动补偿的稳态系数
//   pos_ctrl.alpha=0.95;    //非线性函数的参数
//   pos_ctrl.init(&pos_ctrl);


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


   // 初始化数据记录模块   Initialize DATALOG module
   dlog.iptr1 = &DlogCh1;
   dlog.iptr2 = &DlogCh2;
   dlog.iptr3 = &DlogCh3;
   dlog.iptr4 = &DlogCh4;
   dlog.trig_value = 0.01 ;//CCS每次图像移动的个数  0x1
   dlog.size = 0x400;    //数据的长度
   dlog.prescalar=1;     //CCS图形采样频率=1/prescalar/(1ms)（主中断周期）=1000Hz
   dlog.init(&dlog);


   for(;;)
   {
	   //        CPU_RUN();
	   DC_Link();
	   deal_key();
	   LCD_DIS();

	   //	    TX232_ChuLi();
	   //		OutLoop_Control();


   }

}

void  QEP_Init(void)
{


//    EALLOW;
    EQep1Regs.QUPRD=1500000;		// Unit Timer for 100Hz at 150 MHz SYSCLKOUT

    EQep1Regs.QDECCTL.all=0x00;
    EQep1Regs.QEPCTL.all=0x820a;

//    EQep1Regs.QDECCTL.all=0x400;
//    EQep1Regs.QEPCTL.all=0x900a;

    //EQep1Regs.QEPCTL.all=0x920a;

//    EQep1Regs.QDECCTL.bit.QSRC=0x00;		// QEP quadrature count mode
//    EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
//    EQep1Regs.QEPCTL.bit.PCRM=0x01;  		// PCRM=00 mode - QPOSCNT reset on index event
//    EQep1Regs.QEPCTL.bit.UTE=1; 		// Unit Timeout Enable
//    EQep1Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
//    EQep1Regs.QEPCTL.bit.QPEN=1; 		// QEP enable

    EQep1Regs.QPOSCNT=0;
    EQep1Regs.QPOSINIT=0;
//    TotalPulse=_IQmpy(_IQ(4),LineEncoder);
    EQep1Regs.QPOSMAX= MaxPulses;
    		//(Uint16)(TotalPulse>>15);
    EQep1Regs.QCAPCTL.all=0x8075;

//	EQep1Regs.QCAPCTL.bit.UPPS=5;   	// 1/32 for unit position
//	EQep1Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock
//	EQep1Regs.QCAPCTL.bit.CEN=1; 		// QEP Capture Enable//    EDIS;



}

interrupt void EPWM_1_INT(void)
{

	IPM_BaoHu();
	Show_time++;
	Show_time2++;
	if(Show_time2==1000)//1秒
	{
		Show_time2=0;
		lcd_dis_flag=1;

	}


	Read_key();
	Ad_CaiJi();
	JiSuan_Dl();
	JiSuan_AvgSpeed();

    

if(Run_PMSM==1&&IPM_Fault==0)
{

	//DAC1_out(_iq data);  注意 data 是标么值哦  dac只能输出0到5v正电压 ,如果data小于0 就会输出为0

	//DAC1_out(Speed_Fdb);//输出速度反馈
	//DAC2_out(IQ_Fdb);//输出iQ反馈电流
	//void LuBo(_iq Ch1,_iq Ch2,_iq Ch3,_iq Ch4)//参数都是标么值 Q格式,知道怎么用了吧
	//----ch1--ch2-ch3-ch4

	//LuBo(ia, ib, ic,Speed);


 // DAC1_out(ID_Fdb);//输出iD反馈电流


//=====================================================================================================
//初始位置定位结束，开始闭环控制
//=====================================================================================================
//
//	if(LocationFlag!=LocationEnd)
//	{
//		Modulation=0.95;
//		HallAngle=0;
//		if(GpioDataRegs.GPCDAT.bit.GPIO78) //W
//		{
//			HallAngle+=1;
//		}
//		if(GpioDataRegs.GPCDAT.bit.GPIO77) //V
//		{
//			HallAngle+=2;
//		}
//
//		if(GpioDataRegs.GPCDAT.bit.GPIO76) //U
//		{
//			HallAngle+=4;
//		}
//		switch(HallAngle)
//		{
//		case 5:
//			Position=PositionPhase60;
//			OldRawTheta =SectorWidth*0+SectorWidth/2;
//			break;
//
//		case 1:
//			Position=PositionPhase360;
//			OldRawTheta =SectorWidth*5+SectorWidth/2;
//			break;
//
//		case 3:
//			Position=PositionPhase300;
//			OldRawTheta =SectorWidth*4+SectorWidth/2;
//			break;
//
//		case 2:
//			Position=PositionPhase240;
//			OldRawTheta =SectorWidth*3+SectorWidth/2;
//			break;
//
//		case 6:
//			Position=PositionPhase180;
//			OldRawTheta =SectorWidth*2+SectorWidth/2;
//			break;
//
//		case 4:
//			Position=PositionPhase120;
//			OldRawTheta =SectorWidth+SectorWidth/2;
//			break;
//
//		default:
//			DC_ON_1;
//			Run_PMSM=2;
//			eva_close();
//			Hall_Fault=1;//霍尔信号错误启动停止
//			break;
//		}
//        LocationFlag=LocationEnd;//定位结束
//        EQep1Regs.QPOSCNT=0;
//        PosInit=(float)PosRevScale*(float)OldRawTheta;
//        OldRawTheta=0;
//	}


	if(LocationFlag!=LocationEnd)    //LocationFlag = TRUE 代表转子要开始初始化
	      {
	          if(LockRotorNum < 25000)//35000
	          {
	              LockRotorNum ++;
	              if(LockRotorNum < 9500)//25000
	              {

	                  EPwm1Regs.CMPA.half.CMPA = 1321;
	                  EPwm2Regs.CMPA.half.CMPA = 652;
	                  EPwm3Regs.CMPA.half.CMPA = 652;
	                  /*EPwm1Regs.CMPA.half.CMPA = 2337;
	                  EPwm2Regs.CMPA.half.CMPA = 1412;
	                  EPwm3Regs.CMPA.half.CMPA = 1412;*/

	              }
	              else
	              {
	                  EPwm1Regs.CMPA.half.CMPA = 3375;
	                  EPwm2Regs.CMPA.half.CMPA = 3375;
	                  EPwm3Regs.CMPA.half.CMPA = 3375;
	              }
	          }
	          else
	          {
	              LocationFlag=LocationEnd;//定位结束
                  EQep1Regs.QPOSCNT=0;
                  OldRawTheta=0;
	          }

	      }


//=====================================================================================================
//初始位置定位结束，开始闭环控制
//=====================================================================================================
	else if(LocationFlag==LocationEnd)
	{  

//======================================================================================================
//QEP转子角度计算
//====================================================================================================== 
//		DirectionQep = EQep1Regs.QEPSTS.bit.QDF;
        RawTheta = EQep1Regs.QPOSCNT;

		MechTheta = (float)PosRevScale*(float)RawTheta;  //单位是圈数

//        if(MechTheta>360)
//        {MechTheta=MechTheta-360;}
//         if(MechTheta<-360)
//        {MechTheta=MechTheta+360;}

		ElecTheta = -MechTheta*PolePairs;
	
//		AnglePU =_IQmpy(_IQ(MechTheta),_IQ(PolePairs));
		AnglePU =_IQ(ElecTheta);

	   	Sine = _IQsinPU(AnglePU);
	   	Cosine = _IQcosPU(AnglePU);    

	   	LoopCnt++;

	   	OutLoop_Control();

// 电流内环控制
	    ialfa=ia;
		ibeta=_IQmpy(ia,_IQ(0.57735026918963))+_IQmpy(ib,_IQ(1.15470053837926));

		id = _IQmpy(ialfa,Cosine) + _IQmpy(ibeta,Sine);
		iq = _IQmpy(ibeta,Cosine) - _IQmpy(ialfa,Sine) ;
//======================================================================================================
//IQ电流PID调节控制
//======================================================================================================
		IQ_Ref=IQ_Given;
		IQ_Fdb=iq;

		IQ_Error=IQ_Ref-IQ_Fdb;

		IQ_Up=_IQmpy(IQ_Kp,IQ_Error);
		IQ_Ui=IQ_Ui + _IQmpy(IQ_Ki,IQ_Up) + _IQmpy(IQ_Ki,IQ_SatError);

		IQ_OutPreSat=IQ_Up+IQ_Ui;

		if(IQ_OutPreSat>IQ_OutMax)
			IQ_Out=IQ_OutMax;
		else if(IQ_OutPreSat<IQ_OutMin)
		 	IQ_Out=IQ_OutMin;
		else
			IQ_Out=IQ_OutPreSat;

		IQ_SatError=IQ_Out-IQ_OutPreSat;

		Uq=IQ_Out;

//======================================================================================================
//ID电流PID调节控制
//======================================================================================================
		ID_Ref=ID_Given;
		ID_Fdb=id;

		ID_Error=ID_Ref-ID_Fdb;

		ID_Up=_IQmpy(ID_Kp,ID_Error);
		ID_Ui=ID_Ui+_IQmpy(ID_Ki,ID_Up)+_IQmpy(ID_Ki,ID_SatError);

		ID_OutPreSat=ID_Up+ID_Ui;

		if(ID_OutPreSat>ID_OutMax)
			ID_Out=ID_OutMax;
		else if(ID_OutPreSat<ID_OutMin)
		 	ID_Out=ID_OutMin;
		else
			ID_Out=ID_OutPreSat;

		ID_SatError=ID_Out-ID_OutPreSat;

		Ud=ID_Out;

//======================================================================================================
//IPark变换
//======================================================================================================
		Ualfa = _IQmpy(Ud,Cosine) - _IQmpy(Uq,Sine);
		Ubeta = _IQmpy(Uq,Cosine) + _IQmpy(Ud,Sine);

//======================================================================================================
//SVPWM 实现
//======================================================================================================
        B0=Ubeta;
		B1=_IQmpy(_IQ(0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2
		B2=_IQmpy(_IQ(-0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2

		Sector=0;
		if(B0>_IQ(0)) Sector =1;
		if(B1>_IQ(0)) Sector =Sector +2;
		if(B2>_IQ(0)) Sector =Sector +4;

		X=B0;  // va
		Y=-B2; // 0.8660254 = sqrt(3)/2
		Z=-B1;

		if(Sector==1)
		{
			t1=Z;
			t2=Y;

			if((t1+t2)>_IQ(1))
			{
				t1=_IQdiv(t1, (t1+t2));
				t2=_IQ(1)-t1;
			}

			Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Ta=Tb+t1;
			Tc=Ta+t2;
		}
		else if(Sector==2)
		{
			t1=Y;
			t2=-X;

			if((t1+t2)>_IQ(1))
			{
				t1=_IQdiv(t1, (t1+t2));
				t2=_IQ(1)-t1;
			}

			Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Tc=Ta+t1;
			Tb=Tc+t2;
 		}
		else if(Sector==3)
 	    {
			t1=-Z;
			t2=X;

			if((t1+t2)>_IQ(1))
			{
				t1=_IQdiv(t1, (t1+t2));
				t2=_IQ(1)-t1;
			}

			Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Tb=Ta+t1;
			Tc=Tb+t2;
	    }
	    else if(Sector==4)
	    {
			t1=-X;
			t2=Z;
			if((t1+t2)>_IQ(1))
			{
				t1=_IQdiv(t1, (t1+t2));
				t2=_IQ(1)-t1;
			}

			Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Tb=Tc+t1;
			Ta=Tb+t2;
	    }
	    else if(Sector==5)
	    {
			t1=X;
			t2=-Y;
			if((t1+t2)>_IQ(1))
			{
				t1=_IQdiv(t1, (t1+t2));
				t2=_IQ(1)-t1;
			}

			Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Tc=Tb+t1;
	 		Ta=Tc+t2;
		}
		else if(Sector==6)
		{
			t1=-Y;
			t2=-Z;
			if((t1+t2)>_IQ(1))
			{
				t1=_IQdiv(t1, (t1+t2));
				t2=_IQ(1)-t1;
			}

			Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Ta=Tc+t1;
			Tb=Ta+t2;
		}

		MfuncD1=_IQmpy(_IQ(2),(_IQ(0.5)-Ta));
		MfuncD2=_IQmpy(_IQ(2),(_IQ(0.5)-Tb));
		MfuncD3=_IQmpy(_IQ(2),(_IQ(0.5)-Tc));

//======================================================================================================
//PWM全比较器参数赋值，用于驱动电机
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



interrupt void SCIBRX_ISR(void)     // SCI-B
{
	
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;
    
}


//=================外环控制===================================
void OutLoop_Control(void)
{ 
	float32 ctrl_uk=0;

	if((Run_PMSM!=1) || (IPM_Fault!=0) ||(LocationFlag!=LocationEnd))
		return;

	if (LoopCnt<OutLoopScaler)
		return;

	LoopCnt=0;

//======================================================================================================
//QEP速度计算
//======================================================================================================
	// 旋转方向判定
	DirectionQep = EQep1Regs.QEPSTS.bit.QDF;
	// 计算机械角度
	RawThetaTmp = (float)RawTheta - (float)OldRawTheta;

	if(DirectionQep ==1) //递增计数，代表顺时针；
	{

		if((OldRawTheta>(MaxPulses-1000)) && (RawTheta<1000))
		{
			RawThetaTmp = RawThetaTmp + (float)MaxPulses;
			PosRevCnt++;    //+= TotalCnt
		}
	}
	else //递减计数，代表逆时针   if(DirectionQep ==0)
	{

		if((RawTheta>(MaxPulses-1000)) && (OldRawTheta<1000))
		{
			RawThetaTmp = RawThetaTmp - (float)MaxPulses;
			PosRevCnt-- ;  // -= TotalCnt
		}

	}
        Place_now = (long)RawTheta + ((long)PosRevCnt * (long)MaxPulses);
        PosInRev = (float32)Place_now * (float32)PosRevScale;
        yk = PosInRev;

	speed_pu_m = (float)SpeedScale*(float)RawThetaTmp;
	Speed = _IQ(speed_pu_m);

//	// T法测速
//	if(EQep1Regs.QEPSTS.bit.UPEVNT==1)
//	{
//		if(EQep1Regs.QEPSTS.bit.COEF==0)
//			t2_t1 =  EQep1Regs.QCPRD;
//		else
//			t2_t1 = 0xFFFF;
//
//		if(DirectionQep==1)
//			Speed_rps = SpeedRpsScale /t2_t1;
//		else
//			Speed_rps = -SpeedRpsScale /t2_t1;
//		speed_pu_t=Speed_rps/50;
//
//		EQep1Regs.QEPSTS.all=0x88;
//	}


	OldRawTheta = RawTheta;


	OutLoopCnt++;
        if(PosEnable ==0)
        {
                PosRevCnt=0;
                Place_now=0;
                yk=0;
                PosInRev=0;
//=================速度环 抗饱和PI控制===================================
                if (OutLoopCnt>=SpeedRefScaler)
		{
			OutLoopCnt=0;
//			if (SpdRef == 0)
//				SpdRef = SpeedRef;
//			else
//				SpdRef = 0;
		}

		Speed_Ref=_IQ(SpeedRef);
		Speed_Fdb=Speed;
		Speed_Error = Speed_Ref - Speed_Fdb;
		Speed_Up=_IQmpy(Speed_Kp,Speed_Error);
		Speed_Ui=Speed_Ui + _IQmpy(Speed_Ki,Speed_Up) + _IQmpy(Speed_Ki,Speed_SatError);
		Speed_OutPreSat=Speed_Up+Speed_Ui;

		if(Speed_OutPreSat>MaxOut)
			Speed_Out=MaxOut;
		else if(Speed_OutPreSat<MinOut)
			Speed_Out=MinOut;
		else
			Speed_Out=Speed_OutPreSat;
		Speed_SatError=Speed_Out-Speed_OutPreSat;

		IQ_Given=Speed_Out;


		DlogCh1=(int16)IQ_Given;
		DlogCh2=(int16)Speed;
		DlogCh3=(int16)id;
		DlogCh4=(int16)iq;
		dlog.update(&dlog);

	}
	else
	{

	//=================位置环控制===================================
                if (OutLoopCnt>=PosRefScaler)
                {
                        OutLoopCnt=0;
                        if (pos_ctrl.Ref == 0)
                                pos_ctrl.Ref = PosRef;

                        else
                                pos_ctrl.Ref = 0;


                        pscale=fabsf(2.0f*PosRef);
                        if (pscale<0.5)
                                pscale=1;
                }

                pos_ctrl.Fdb = yk;
                pos_ctrl.calc(&pos_ctrl);
                ctrl_uk = pos_ctrl.Out;   //UMAX;

//		float32 err = pos_ctrl.Ref - pos_ctrl.Fdb;
//		float32 deadband = 1.0f / (float)MaxPulses * 2; // 例如 2 个脉冲的范围
//		if (fabs(err) < deadband) {
//		    ctrl_uk = 0; // 或者一个小的保持电流
//		}

		IQ_Given = _IQdiv(_IQ(ctrl_uk),_IQ(UMAX));
		if (IQ_Given>MaxOut)
			IQ_Given=MaxOut;
		else if(IQ_Given<MinOut)
				IQ_Given=MinOut;

                performance_metrics_update(yk);

		DlogCh1=(int16)IQ_Given;  //_IQ15(ctrl_uk)
		DlogCh2=(int16)_IQ15(pos_ctrl.Fdb/pscale);
		DlogCh3=(int16)_IQ15(pos_ctrl.vk/50);
		DlogCh4=(int16)_IQ15(pos_ctrl.dk/UMAX);
		dlog.update(&dlog);


	}


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
