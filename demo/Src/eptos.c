/*=====================================================================================
File name:      PTOS.c  (float version)

*** Proximate Time Optimal Servo Controller (PTOS) with a linear extended state observer  ***
                    
Originator: Control Systems Group, FZU

重要说明：这个算法的输入输出量都是以实际的物理量单位，而不是pu值；
使用方法：1. 在主程序中先声明一个PTOC对象，设置系统模型参数和控制器参数之后，调用ptoc_init；
       2. 在每个运行周期（中断程序）中，设置当前的输入量r(k)和y(k), 调用ptoc_calc,得到u(k) .

=====================================================================================

-------------------------------------------------------------------------------------*/
//#define GLOBAL_Q 24
//#include "IQmathLib.h"    //  Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file
//#include "dmctype.h"
#include "ptos.h"
#include  <math.h>
//#include  <stdlib.h>
//#include "DSP2833x_Examples.h"
#define PI 3.1415926

void ptoc_init(PTOC *v)
{

    float32 b,z0,w0;
    b = v->b;

    // for cptos law, a positive omega is specified to initialize --Scheme 2
    v->k1 = v->omega*v->omega/b;
    v->k2 = (2*v->zeta*v->omega)/b;

//  //w0 = b*v->umax*(-v->k2*b)/(a*-v->k2*b)+v->omega*v->omega;
//  w0 =2*v->zeta*v->omega+a;
//  v->yl =  b*v->umax*w0/(a*w0+v->omega*v->omega);
//  v->vs =  b*v->umax*log(1-a*v->yl/(b*v->umax))/(a*a)-(v->yl*b*v->umax)/(a*(a*v->yl-b*v->umax));

    v->vs = b*v->alpha*v->umax*v->zeta/v->omega;
    v->yl = 2*v->vs*v->zeta/v->omega;

    z0=v->zeta0;
    w0=v->omega0;

//  v->L1=a+2*z0*w0;
//  v->L2=w0*w0/b;

    v->L1=2*z0*w0;
    v->L2=w0*w0/b;

//    v->A11=1-2*z0*w0*T;    // for matrix Ac
//    v->A12=b*T;
//    v->A21=-w0*w0/b*T;
//    v->A22=1;

    v->A11=-2*z0*w0;    // for matrix Ac
    v->A12=b;
    v->A21=-w0*w0/b;
    v->A22=0;

//    v->B11=v->A12;   // for matrix Bc * (u; y)
//    v->B12=((1-4*z0*z0)*w0-2*z0*a)*w0*T;
//    v->B21=0;
//    v->B22=-(a+2*z0*w0)*w0*w0*T/b;

    v->B11=b;   // for matrix Bc * (u; y)
    v->B12=(1-4*z0*z0)*w0*w0;
    v->B21=0;
    v->B22=-2*z0*w0*w0*w0/b;


    v->sts=1;

}


void ptoc_calc(PTOC *v)
{
    float32 rk,yk,ek,vk,dk;
    float32 xc1,xc2,uk,umax,fpe;


    yk = v->Fdb;           // system output y(k)
    rk = v->Ref;
    //the estimated values
    //vk = v->speed;

    vk = v->xc1+v->L1*yk;        //vk=v->xc1+v->L1*yk;
    dk = v->xc2+v->L2*yk;        //dk=v->xc2+v->L2*yk;

    umax = v->umax;

    v->vk = vk;
    v->dk =dk;// dk*0.0014286;

    if(v->Ref!=v->sts)
    {
        v->sts=v->Ref;

        ptoc_init(v);
    }


    ek = rk-yk;    // tracking error e(k)=r(k)-y(k)
    v->ek = ek;;
    if (fabs(ek) <= v->yl)//v->vl)
    {
        uk = v->k1*ek - v->k2*vk-v->fd*v->dk;
    }

    else
    {
        //fpe = bu*log(fabs(1-fabs(vk)/bu))/v->a - v->vs;     // fp(e)
        fpe = sqrt(2 * v->b * v->alpha * umax * fabs(ek)) - v->vs;     // fp(e)

        if (ek<0)
            {
            fpe = -fpe;
            }
        uk = v->k2*(fpe-vk)- v->fd*dk;
    }



    if (uk > umax)     // saturation
        uk = umax;
    else if (uk < -umax)
     uk = -umax;

    v->Out = uk;   // output of control signal

    // update the observer state  for the next cycle.
    xc1 = v->xc1 + v->Ts * (v->A11*v->xc1 + v->A12*v->xc2 + v->B11*uk + v->B12*yk);
    xc2 = v->xc2 + v->Ts * (v->A21*v->xc1 + v->A22*v->xc2 + v->B21*uk + v->B22*yk);

    v->xc1 = xc1;
    v->xc2 = xc2;

}



