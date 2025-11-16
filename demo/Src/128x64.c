#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File


Uint16 addr_tab[]={
char Lcd_Dat[6]={0};
    EALLOW;
    GpioCtrlRegs.GPBMUX1.bit.GPIO45=0;//sdl_lcd
     GpioCtrlRegs.GPBDIR.bit.GPIO45=1;
     
     GpioCtrlRegs.GPBMUX1.bit.GPIO44=0;//scl_lcd
     GpioCtrlRegs.GPBDIR.bit.GPIO44=1;
     EDIS;

     SCL_LCD_0;
{
    Uint16 i=0;  

	for(i=0;i<N_US;i++)
	{
	  asm("	NOP");
	
	}

}

void Init_lcd_Gpio(void)
{
	EALLOW;
	GpioCtrlRegs.GPBMUX1.bit.GPIO45=0;//sdl_lcd
	GpioCtrlRegs.GPBDIR.bit.GPIO45=1;

	GpioCtrlRegs.GPBMUX1.bit.GPIO44=0;//scl_lcd
	GpioCtrlRegs.GPBDIR.bit.GPIO44=1;
	EDIS;

	SCL_LCD_0;

}





void LCD_SendByte(Uint16 dat)  //
{
    Uint16 i=0;
    EALLOW;
    GpioCtrlRegs.GPBDIR.bit.GPIO45=1;//shuchu
    EDIS;
    for(i=0;i<8;i++)
    {
        SCL_LCD_0;
        iic_delay();
        if( dat & 0x0080 ) 
		{
		    SDL_LCD_1;
		}
		else 
		{
		    SDL_LCD_0;
		}

        SCL_LCD_1;
        iic_delay();
        dat <<= 1;
        SCL_LCD_0;
		

    }
	
 
}

void Lcd_WriteCmd(Uint16 dat)
{
     
     //CheckBusy();
     DelayUS2(10000);
     LCD_SendByte(0xfa);              //11111,RW(0),RS(1),0
void hanzi_Disp(Uint16 x,Uint16 y,char  *s)
 Lcd_WriteCmd(addr_tab[8*x+y]);  //дַ
 while(*s>0)
    {
  Lcd_WriteData(*s);    //д
  DelayUS2(1);
  s++;   
    } 
    Lcd_WriteCmd(0x30);          //ѡָ
  hanzi_Disp(0,0,"ͬʵ");
  
   
  hanzi_Disp(1,0,"UDC:000 RPM:0000");
  
   
  hanzi_Disp(2,0,"IA:00.0 IB:00.0");
   
   
  hanzi_Disp(3,0,"+000  ֹͣ  ");
	
    i= U_dc_dis/100;//
    Lcd_Dat[0]=0x30+i;

    
    Lcd_Dat[0]=0x30+i;//ǧ
            
    Lcd_Dat[3]=0x30+i; 
    Uint16 i=0;

    Lcd_Dat[0]=':';
    i= I_A/100;//
    Lcd_Dat[1]=0x30+i;

    i= (I_A/10)%10;//ʮ
    Lcd_Dat[2]=0x30+i;

    Lcd_Dat[3]='.';

    i= I_A%10;//
    Lcd_Dat[4]=0x30+i;
    
    Lcd_Dat[5]='\0';


     Lcd_Dat[0]=':';
    i= I_B/100;//
    Lcd_Dat[1]=0x30+i;


    

    if(ZhengFan==1)
         Lcd_Dat[0]='+';//ת

         Lcd_Dat[0]='-';//ת
        
    i= speed_give/100;//
    Lcd_Dat[1]=0x30+i;

    i= (speed_give/10)%10;//ʮ
    Lcd_Dat[2]=0x30+i;
    i= speed_give%10;//
    Lcd_Dat[3]=0x30+i;
    Lcd_Dat[4]='\0';
    hanzi_Disp(3,0,Lcd_Dat);//ʾ趨ת

     if(Run_PMSM==1)
    {
        hanzi_Disp(3,3,"");

    }
     else
     {
          hanzi_Disp(3,3,"ֹͣ");

     }


       if(IPM_Fault==1)
         hanzi_Disp(3,6,"");

    else if(DC_ON_OPEN==1)
         hanzi_Disp(3,6,"");

     else if(DC_ON_OPEN==2)
         hanzi_Disp(3,6,"");

     else if(DC_ON_OPEN==3)
         hanzi_Disp(3,6,"ѹ");

       else if(DC_ON_OPEN==4)
    {
         hanzi_Disp(3,6,"");

    }
     else if(O_Current==1)
     {
        hanzi_Disp(3,6,"");
         
     }
     else if(O_Current==2)
     {
        hanzi_Disp(3,6,"");
         
     }
     else if(Hall_Fault==1)
     {
        hanzi_Disp(3,6,"");

     }
     else if(ShangDian_Err==1)
     {

        hanzi_Disp(3,6,"");//ϵʱ


     }
     else
     {
        hanzi_Disp(3,6,"");

     }
       
void LCD_DIS(void)
{
    static Uint16 i = 0;

    if (lcd_dis_flag == 1)
    {
        if (i == 0)
        {
            Lcd_Dis_Udc();
            i++;
            lcd_dis_flag = 0;
        }
        else if (i == 1)
        {
            Lcd_Dis_RPM();
            i++;
            lcd_dis_flag = 0;
        }
        else if (i == 2)
        {
            Lcd_Dis_DL();
            i++;
            lcd_dis_flag = 0;
        }
        else if (i == 3)
        {
            Lcd_Dis_sheding();
            i = 0;
            lcd_dis_flag = 0;
        }
    }
}
	{
//		sprintf(Lcd_Dat,"%5.2f",SpeedRef);     // 这个语句 运行会出错
		if (SpeedRef>0)
			Lcd_Dat[0]='+';
		else
			Lcd_Dat[0]='-';

		i=(int)fabs(SpeedRef);
		Lcd_Dat[1]=0x30+i;
		Lcd_Dat[2]='.';
		i=(int)(fabs(SpeedRef*10))%10;
		Lcd_Dat[3]=0x30+i;
		i=(int)(fabs(SpeedRef*100))%10;
		Lcd_Dat[4]=0x30+i;
	}
	else
	{
//		sprintf(Lcd_Dat,"P%4.1f",PosRef);   // 这个语句 运行会出错
		Lcd_Dat[0]='P';
		if (PosRef>0)
			Lcd_Dat[1]=' ';
		else
			Lcd_Dat[1]='-';
		i=(int)fabs(PosRef);
		Lcd_Dat[2]=0x30+i;
		Lcd_Dat[3]='.';
		i=(int)fabs(PosRef*10)%10;
		Lcd_Dat[4]=0x30+i;
	}


    Lcd_Dat[5]='\0';
    hanzi_Disp(3,0,Lcd_Dat);//显示设定转速或位置


    if(Run_PMSM==1)
    {
    	hanzi_Disp(3,3,"运行");
    }
    else
    {
    	hanzi_Disp(3,3,"停止");
    }


    if(IPM_Fault==1)
    {
    	hanzi_Disp(3,6,"故障");
    }
    else if(DC_ON_OPEN==1)
    {
    	hanzi_Disp(3,6,"主电");
    }
    else if(DC_ON_OPEN==2)
    {
    	hanzi_Disp(3,6,"掉电");
    }
    else if(DC_ON_OPEN==3)
    {
    	hanzi_Disp(3,6,"过压");
    }
    else if(DC_ON_OPEN==4)
    {
    	hanzi_Disp(3,6,"仿真");
    }
    else if(O_Current==1)
    {
    	hanzi_Disp(3,6,"过流");
    }
    else if(O_Current==2)
    {
    	hanzi_Disp(3,6,"限流");
    }
    else if(Hall_Fault==1)
    {
    	hanzi_Disp(3,6,"霍尔");
    }
    else if(ShangDian_Err==1)
    {
    	hanzi_Disp(3,6,"保护");//上电时序错误
    }
    else
    {
    	hanzi_Disp(3,6,"正常");
    }




}




void LCD_DIS(void)
{
static Uint16 i=0;
if(lcd_dis_flag==1)
{
    if(i==0)
    {
    Lcd_Dis_Udc();
    i++;
    lcd_dis_flag=0;
    }
    else if(i==1)
    {
    Lcd_Dis_RPM();
    i++;
    lcd_dis_flag=0;
    }
    else if(i==2)
    {
    Lcd_Dis_DL();
    i++;
    lcd_dis_flag=0;
    }
    else if(i==3)
    {
    Lcd_Dis_sheding();
    i=0;
    lcd_dis_flag=0;
    }

}


}



	
//===========================================================================
// End of file.
//===========================================================================

