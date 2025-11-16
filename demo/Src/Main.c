//*****************************************************************************************************
//FlashRAM汾л˵(ĬΪram汾)
//
//һ.лΪFlashд汾
//1.е:MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//               InitFlash();
//  ȡע
//2.е28335_RAM_lnk.cmdӹɾCMDļµF28335.cmdļȫһμд
//
//.лΪRAM߷汾
//1.е:MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//               InitFlash();
//  ע͵
//2.еF28335.cmdӹɾCMDļµ28335_RAM_lnk.cmdļȫһμɡ
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

interrupt void EPWM_1_INT(void);                //תӳʼ㷨SVPWMʵ
interrupt void INT3_ISR(void);                  // жϺ12Ӧλ
void Init_SiShu(void);                          //ʼ
void performance_metrics_update(float y);
//RCNS pos_ctrl = RCNS_DEFAULTS;
PTOC pos_ctrl = PTOC_DEFAULTS;                  //ṹPTOCͱʼ

DLOG_4CH dlog = DLOG_4CH_DEFAULTS;              //ṹDLOGͱͼʾָ뺯


//*****************************************************************************************************
//ȫֱʼ
//***************************************************************************************************** 

float32 i=0;
float32 j=0;
float32 k=0;
Uint16 T1Period=0;     // T1ʱ(Q0)
Uint16 T3Period = 0;
float32 Modulation=0.25;    // Ʊ
int16 MPeriod=0;
int32 Tmp=0;
_iq PolePairs=_IQ(4); 

//:::::::::::::::::::::::::::λû:::::::::::::::::::::::::::
long Place_now=0;//λñֵ
Uint16 PlaceSetBit=0;  //λ趨־λ
Uint16 PosEnable=1;//λÿ ʹ  1 ʹ ;  0 -> 
int32 PosCount = 0;
float32 pscale=2;

//===============תӳʼλöλ=============================  
Uint16 LocationFlag=TRUE;
Uint16 LocationEnd=0;
Uint16 Position=1;
Uint16 PositionPhase60=1;
Uint16 PositionPhase120=2;
Uint16 PositionPhase180=3;
Uint16 PositionPhase240=4;
Uint16 PositionPhase300=5;
Uint16 PositionPhase360=6;

//===============DACģ===================================== 
_iq DACTemp0=0;
_iq DACTemp1=0;
_iq DACTemp2=0; 

_iq MfuncC1=0;
_iq MfuncC2=0;
_iq MfuncC3=0; 
Uint16 ZhengFan=1;  
 
//===============תٶȼ===================================== 
Uint16 SpeedLoopPrescaler = 20;     // ٶȻ
Uint16 SpeedLoopCount = 1;          // ٶȻ
_iq NewRawTheta=0;
_iq OldRawTheta=0;
_iq SpeedRpm=0;                     //ٶȣλת/ÿ
Uint16 Hall_Fault=0;
_iq RawThetaTmp=0;
float32 SpeedRef=0;
_iq Speed=0;                        //ٶȣֵ

//===============תӽǶȼ===================================
Uint16 DirectionQep=0;               //תת
_iq RawTheta=0;
_iq OldRawThetaPos = 0;

_iq TotalPulse=0; 

_iq MechTheta = 0;                   //еǶȣλ
_iq ElecTheta = 0;                   //Ƕȣλ
_iq AnglePU=0;                       //Ƕȱۻ
_iq Cosine=0;
_iq Sine=0;


//=========================================== 
_iq ia=0;
_iq ib=0;
_iq ic=0;
_iq ialfa=0;
_iq ibeta=0; 
_iq id=0;
_iq iq=0; 

//===============PI============================ 
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

_iq Speed_Given=_IQ(0.2); //ٶȸ    ֵ 0.2==>600RPMת1.0==>3000RPM
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

//===============SVPWM==================================== 
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


//========================ٶȻPI=================================
_iq Speed_Kp=_IQ(2.5);
_iq Speed_Ki=_IQ(0.007);
_iq Speed_Kd=_IQ(0);
//=====================================================================

//========================QPI==============================
_iq IQ_Kp=_IQ(0.1221);
_iq IQ_Ki=_IQ(0.061);
//=====================================================================

//========================DPI==============================
_iq ID_Kp=_IQ(0.1221);
_iq ID_Ki=_IQ(0.061);
//=====================================================================

long PlaceSet=10000;//λû
Uint16 PlaceEnable=0;//λûʹ  1 ʹ ;  0 ֹ

//=============================================================
float32 E_Ding_DianLiu=4.0;        //õĶЧֵ  λA
Uint16 BaseSpeed=3000;              //õת

                
/*****ͼʾ****************/
int16 DlogCh1=0;
int16 DlogCh2=0;
int16 DlogCh3=0;
int16 DlogCh4=0;

/*****תӳʼ****************/
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
float32 sample_period = 0.02;   // ڣ

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
  
   InitEPwm_1_2_3();        //pwmʼ
   QEP_Init();              //qepʼ

   Init_SiShu();
   ADC_Soc_Init();
   
 
   eva_close();             //λؼƱ
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   

   if(AD_BUF[7]<150)
   {
       Pwm_EN_0;//PWMʹ

   }
   else
   {
       Pwm_EN_1;//ֹPWMʹ
       ShangDian_Err=1;

   }
    
   // ʼݼ¼ģ   Initialize DATALOG module
    dlog.iptr1 = &DlogCh1;
    dlog.iptr2 = &DlogCh2;
    dlog.iptr3 = &DlogCh3;
    dlog.iptr4 = &DlogCh4;
    dlog.trig_value = 1 ;   //CCSÿͼƶĸ  0x1
    dlog.size = 0x400;     //ݵĳȣһɼ 1024 
    dlog.prescalar=3;      //CCSͼβƵ=1/prescalar/(1ms)жڣ=1000Hz
    dlog.init(&dlog);


//    //eptosɵĳʼ
//    pos_ctrl.umax=UMAX;
//    pos_ctrl.Ts=0.002;
//    pos_ctrl.b=500;  // bԽ󣬳ԽСԽƽ     //ģͲ
//    pos_ctrl.a=-3;  // a<=0;
//    pos_ctrl.zeta=0.85;  //ԿƵϵ
//    pos_ctrl.omega=60;    //ԿƵȻƵ
//    pos_ctrl.zeta0=0.8;  //۲ϵ 0.7071
//    pos_ctrl.omega0=200;    //۲ȻƵ
//    pos_ctrl.fd=0.5;    //Ŷ̬ϵ
//    pos_ctrl.alpha=0.95;    //ԺĲ
//    pos_ctrl.init(&pos_ctrl);

    //PTOSɵĳʼ
    pos_ctrl.umax=UMAX;
    pos_ctrl.Ts=0.002;   // 0.001 or 0.002
    //      pos_ctrl.a=-3;
    pos_ctrl.b=500;  //ģͲ
    pos_ctrl.a=-3;
    pos_ctrl.alpha=0.95;  //
    pos_ctrl.zeta=0.85;  //  ϵ
    pos_ctrl.omega=60;//80;    //ȻƵ
    pos_ctrl.zeta0=0.80;  //۲ϵ
    pos_ctrl.omega0=120;//62;    //۲ȻƵ
    pos_ctrl.gama=0.85;//
    pos_ctrl.lambda=3;
    pos_ctrl.fd=0.3;    //Ŷ̬ϵ
    pos_ctrl.init(&pos_ctrl);   //óʼ


   DELAY_US(1000000);
    
   
   IER |= M_INT3;
   IER |= M_INT9;
   IER |= M_INT12;

   //жʹ
   //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//timer0
   PieCtrlRegs.PIEIER3.bit.INTx1=1;//epwm1int
   PieCtrlRegs.PIEIER9.bit.INTx3=1;//scib
   PieCtrlRegs.PIEIER12.bit.INTx1=1;//xint3
    
   Init_lcd();//ʼʾ
 
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

       //SpeedRef = 0.1;

   for(;;)
     {
        //CPU_RUN();
        DC_Link();
        deal_key();//ж϶İжϣһϵеĲ
        LCD_DIS();// LCD ʾͬϢ
        //TX232_ChuLi();

     }

}

interrupt void EPWM_1_INT(void)
{
       _iq t_01,t_02;
       //IPM_BaoHu();
    
   
  Read_key();//ȡϢ------жĸ
  Ad_CaiJi(); //ɼia,ib,ic;ںClark,Park仯һõiqid
  //JiSuan_Dl();//Բɼĵм㣬жǷ


if(Run_PMSM==1&&IPM_Fault==0)
{

     if(LocationFlag == TRUE)    //תҪʼʼ
      {
          if(LockRotorNum < 25000)//35000
          {
              LockRotorNum ++;
              if(LockRotorNum < 9500)       //25000λת
              {
                  //ֱPWMıȽϼĴCMPAֵ--------PWMռձȲռѹʸʹתӵָλ
                  EPwm1Regs.CMPA.half.CMPA = 1321;
                  EPwm2Regs.CMPA.half.CMPA = 652;
                  EPwm3Regs.CMPA.half.CMPA = 652;
                  /*EPwm1Regs.CMPA.half.CMPA = 2337;
                  EPwm2Regs.CMPA.half.CMPA = 1412;
                  EPwm3Regs.CMPA.half.CMPA = 1412;*/

              }
              else                          //סתӡֹŶ
              {
                  EPwm1Regs.CMPA.half.CMPA = 3375;
                  EPwm2Regs.CMPA.half.CMPA = 3375;
                  EPwm3Regs.CMPA.half.CMPA = 3375;
              }
          }
          else
          {
              EQep1Regs.QPOSCNT = 0;//λ
              LocationFlag = FALSE; //LocationFlag = FALSE תӳʼϣʼջ
              Modulation=0.95;// PWM  95%
          }

      }

        

//=====================================================================================================
//ʼλöλʼջ
//=====================================================================================================
    if(LocationFlag == FALSE)
    {
//======================================================================================================
//QEPǶȼ
//====================================================================================================== 

// תж 
        DirectionQep = EQep1Regs.QEPSTS.bit.QDF;//QDFDirection Flagλʾתӵת

        RawTheta = _IQ(EQep1Regs.QPOSCNT);//ȡQEPԭʼֵ

        if(DirectionQep ==1) //˳ʱ룻
        {
            if((OldRawThetaPos>_IQ(3900)) && (RawTheta<_IQ(900)))   //ӽӽֵ3900ƵСֵС900ʱʾתתһȦ
            {
                PosCount += TotalCnt;
            }
            Place_now= _IQtoF(RawTheta)+PosCount;
            OldRawThetaPos = RawTheta;
        }

        else if(DirectionQep ==0)//ݼʱ
        {
            if((RawTheta>_IQ(3000)) && (OldRawThetaPos<_IQ(1000)))
            {
                PosCount -= TotalCnt;
            }
            Place_now = _IQtoF(RawTheta)+PosCount;
            OldRawThetaPos = RawTheta;
        }

        MechTheta = _IQmpy(2949,RawTheta);//2949Q15ʽתΪͼΪ2949/32768=0.09һQCLKĽǶΪ360/4000=0.09

        //ΪֹеǶȳ360Ȼ߸360
        if(MechTheta>_IQ(360))
        {MechTheta=MechTheta-_IQ(360);}
        if(MechTheta<_IQ(-360))
        {MechTheta=MechTheta+_IQ(360);}
        //Ͻֹ

        //ɻеǶȵóǶ,עǶǿԴ360ΪIQ15ֻܱʾ-1-1֮䣬ԶȡС֣ҲʹõǶС360. ںparkȱ仯
        ElecTheta = _IQmpy(PolePairs,MechTheta);


        AnglePU=_IQdiv(ElecTheta,_IQ(360)); //Ƕȵıۻ
        Sine = _IQsinPU(AnglePU);           // ǶȶӦֵ
        Cosine = _IQcosPU(AnglePU);         // ǶȶӦֵ


//======================================================================================================
//QEPٶȼ
//====================================================================================================== 

        if (SpeedLoopCount>=SpeedLoopPrescaler)//жǷﵽִλÿƼʱ
        {
            OldRawTheta = NewRawTheta;//һڵıλ
            SpeedLoopCount=1;
            RawThetaTmp=0;

//=================λÿ(ָģԼ)===================================

            //λÿƲο pos_ctrl.Ref  0  PosRef ֮л
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

            //ѱλֵתɻеǶȻ߱λ÷
             yk= Place_now*0.00025;//(Place_now*0.09)/360 = 0.00025;
                           //rk = v_Ref * 2 * pi;
                           //c_count = sin( 5 * v_tk);
                           //rk = pi * c_count;



             //pos_ctrl.vk=p_Speed/60;
             pos_ctrl.Fdb = yk;             // λ÷ֵǰתӵλ
             pos_ctrl.calc(&pos_ctrl);      // λÿ㺯ݲοͷ
             ctrl_uk = pos_ctrl.Out;        // UMAX;ȡλÿֵ

             IQ_Given =_IQdiv(_IQ(ctrl_uk),_IQ(UMAX));  //λÿźŹһ
             if (IQ_Given>MaxOut)
                 IQ_Given=MaxOut;
             else if(IQ_Given<MinOut)
                 IQ_Given=MinOut;

             performance_metrics_update(yk);

//=================ʾģ===================================

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


        //Clarke任 + Park任 ϵļʽ
        ialfa=ia;
        ibeta=_IQmpy(ia,_IQ(0.57735026918963))+_IQmpy(ib,_IQ(1.15470053837926));

        id = _IQmpy(ialfa,Cosine) +_IQmpy(ibeta,Sine);
        iq = _IQmpy(ibeta,Cosine)- _IQmpy(ialfa,Sine);


//======================================================================================================
//IQPIDڿ
//======================================================================================================
        IQ_Ref=IQ_Given;
        IQ_Fdb=iq;

        IQ_Error=IQ_Ref-IQ_Fdb;

        IQ_Up=_IQmpy(IQ_Kp,IQ_Error);           //
        IQ_Ui=IQ_Ui + _IQmpy(IQ_Ki,IQ_Up);      //Ա

        IQ_OutPreSat=IQ_Up+IQ_Ui;

        if(IQ_OutPreSat>IQ_OutMax)
            IQ_Out=IQ_OutMax;
        else if(IQ_OutPreSat<IQ_OutMin)
            IQ_Out=IQ_OutMin;
        else
            IQ_Out=IQ_OutPreSat;


        Uq=IQ_Out;

//======================================================================================================
//IDPIDڿ
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
//IPark任
//====================================================================================================== 
        Ualfa = _IQmpy(Ud,Cosine) - _IQmpy(Uq,Sine);
        Ubeta = _IQmpy(Uq,Cosine) + _IQmpy(Ud,Sine);

//======================================================================================================
//SVPWMʵ
//====================================================================================================== 
        //ྲֹϵ-£еĵѹ Ualfa, Ubeta ͶӰϵ B0B1B2 
        B0=Ubeta;
        B1=_IQmpy(_IQ(0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta);// 0.8660254 = sqrt(3)/2
        B2=_IQmpy(_IQ(-0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2

        //ϵж
        Sector=0;
        if(B0>_IQ(0)) Sector =1;
        if(B1>_IQ(0)) Sector =Sector +2;
        if(B2>_IQ(0)) Sector =Sector +4;

        //ڼռձ
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

        //  PWM ռձʼʱתΪĶ PWM Ƚ----[-1,1]
        MfuncD1=_IQmpy(_IQ(2),(_IQ(0.5)-Ta));
        MfuncD2=_IQmpy(_IQ(2),(_IQ(0.5)-Tb));
        MfuncD3=_IQmpy(_IQ(2),(_IQ(0.5)-Tc));

//======================================================================================================
//EVAȫȽֵ
//====================================================================================================== 
    MPeriod = (int16)(T1Period * Modulation);              // Q0 = (Q0 * Q0)

    Tmp = (int32)MPeriod * (int32)MfuncD1;                    // Q15 = Q0*Q15ȫȽCMPR1ֵ
     EPwm1Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    Tmp = (int32)MPeriod * (int32)MfuncD2;                    // Q15 = Q0*Q15ȫȽCMPR2ֵ
     EPwm2Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

    Tmp = (int32)MPeriod * (int32)MfuncD3;                    // Q15 = Q0*Q15ȫȽCMPR3ֵ
     EPwm3Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

         
    }
    }


if(DC_ON_flag==1)
{

        if(U_dc_dis<10)//ִͣ
        {
        eva_close();
        Run_PMSM=2;
        DC_ON_flag=0;

        }

}


EPwm1Regs.ETCLR.bit.INT=1;//жϱ־λ
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

    // 㳬ֵʾֵ0
    float os = (y_max - ref) / ref;
    if(os < 0) os = 0;
    overshoot = os;
    y_max = 0;
//    // ̬ֵ
//    steady_state_error = fabsf(y - ref);

    // ¼ʱ䣬״γ0.9*ref
    if(!rise_flag && y >= 0.9f * ref && y>1e-6)
    {
        rise_time = sample_count * sample_period;
        rise_flag = 1;
    }


}

//===========================================================================
// No more.
//===========================================================================
