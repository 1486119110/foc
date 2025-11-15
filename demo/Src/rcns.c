/*=====================================================================================
  File name:  rcns.c  (float version)

*** Robust Nonlinear Servo Controller (RCNS) with a linear extended state observer  ***
                    
   Originator:	Control Systems Group, FZU

   重要说明：这个算法的输入输出量都是以实际的物理量单位，而不是pu值；
   使用方法：1. 在主程序中先声明一个RCNS对象，设置系统模型参数和控制器参数之后，调用rcns_init；
        2. 在每个运行周期（中断程序）中，设置当前的输入量r(k)和y(k), 调用rcns_calc,得到u(k) .
			
=====================================================================================
 History:07-17-2012	Version 1.0
 History:01-05-2013	Version 2.0  float32 version
 History:01-25-2018	Version 3.0  parameterized version
-------------------------------------------------------------------------------------*/
#include "rcns.h"
#include <math.h>


void rcns_init(RCNS *v)
{
	float32 a,b,T,zeta,omega;

	a=v->a;
	b=v->b;
	T=v->Ts;

	// for control law
	zeta=v->zeta;
	omega=v->omega;

	v->f1=-omega*omega/b;
	v->f2=-(a+2*zeta*omega)/b;
	v->fr=-v->f1;
//	v->fd=0;

	v->fn1=omega*omega/b;
	v->fn2=omega/(b*zeta);

	// for observer
	zeta=v->zeta0;
	omega=v->omega0;

	v->L1=a+2*zeta*omega;
	v->L2=omega*omega/b;

	v->A11=1-2*zeta*omega*T;    // for matrix Ac
	v->A12=b*T;
	v->A21=-omega*omega/b*T;
	v->A22=1;

	v->B11=v->A12;   // for matrix Bc * (u; y)
	v->B12=((1-4*zeta*zeta)*omega-2*zeta*a)*omega*T;
	v->B21=0;
	v->B22=-(a+2*zeta*omega)*omega*omega*T/b;

//	v->xc1=0;
//	v->xc2=0;
}

void rcns_calc(RCNS *v)
{
	float32 yk,rk,uk=0,xc1=0,xc2=0;
	float32 ek,alpha1;

	yk = v->Fdb ;           // system output y(k)
	rk = v->Ref ;

	//the estimated values
	v->vk = v->xc1 + v->L1 * yk;          //vk=v->xc1+v->L1*yk;
	v->dk = v->xc2 + v->L2 * yk;          //dk=v->xc2+v->L2*yk;
	v->ek = rk-yk;
	ek = v->ek;               // tracking error e(k)=r(k)-y(k)

	if(v->Ref!=v->y0)
	{
		v->y0=v->Ref;
		if (fabs(ek)>v->margin)	//  if fabs(ek)>0
			v->alpha0=1.0/fabs(ek);  // 1/|e(0)|
		else
			v->alpha0=1;

	     rcns_init(v);
	}

	alpha1 = v->alpha * fabs(ek) * v->alpha0;
//	v->rho = v->beta * (PI/2 - atan(alpha1));
    v->rho = v->beta /(1+ alpha1);
	//////////////////////
	uk = (v->f1 * yk) + (v->f2 * v->vk) + (v->fr * rk);  //linear control law
	uk = uk - v->rho * (v->fn1 * (yk - rk) + v->fn2 * v->vk);
	v->uk = uk - v->dk * v->fd;    // disturbance compensation

	// saturation
	if (v->uk > v->umax)
		v->uk = v->umax;
	else if (v->uk < -v->umax)
		v->uk = -v->umax;

	// output of control signal
	v->Out = v->uk;

	// update the observer state  for the next cycle.
	xc1 = v->A11 * v->xc1 + v->A12 * v->xc2 + v->B11 * v->uk + v->B12 * yk;
	xc2 = v->A21 * v->xc1 + v->A22 * v->xc2 + v->B21 * v->uk + v->B22 * yk;
	v->xc1 = xc1;
	v->xc2 = xc2;
}

