/*=====================================================================================
File name:      PTOS.c  

*** Proximate Time Optimal Servo Controller (PTOS) with a linear extended state observer  ***
                    
Originator: Control Systems Group, FZU

重要说明：这个算法的输入输出量都是以实际的物理量单位，而不是pu值；
使用方法：1. 在主程序中先声明一个PTOC对象，设置系统模型参数和控制器参数之后，调用ptoc_init；
          2. 在每个运行周期（中断程序）中，设置当前的输入量r(k)和y(k), 调用ptoc_calc,得到u(k) .  

=====================================================================================
 History:7-17-2012  Version 1.0
 History:5-22-2013  Version 3.0   revised for speed constraint
 History:1-30-2021  Version 4.0   revised for varying speed constraint
 History:9-11-2024  Version 5.0   revised for using finite-time control to replace linear control
 -------------------------------------------------------------------------------------*/
#include  <math.h>
#include "vptos.h"

float32 fal(float32 fep, float32 alpha, float32 delta);
float32 sig(float32 a, float32 gama);

void ptoc_init(PTOC *v)
{

    float32 b,w0,z0;

    b = v->b;

    z0=v->zeta0;
    w0=v->omega0;
    // a positive omega is specified to initialize

    v->k1 = v->omega*v->omega/b;
    v->k2 = 2*v->zeta*v->omega/b;
    v->k3 = v->k1*pow(v->yl,v->gama/(2-v->gama))/pow(v->vs,v->gama);

    v->vs = b*v->alpha*v->umax*v->zeta/v->omega;
    v->yl = 2*v->vs*v->zeta/v->omega;


    v->L1 = (1+2*z0)*w0;
    v->L2 = (1+2*z0)*w0*w0;
    v->L3 = w0*w0*w0;

}

void ptoc_calc(PTOC *v)
{
    float32 rk,yk,ek,vk,ep;
    float32 uk,fqe,umax,lambda,gama;

    if (v->Ref!=v->Ref1)    // when the set-point r is changed
    {
      ptoc_init(v);
      v->Ref1 = v->Ref;
    }

    yk = v->Fdb;           // system output y(k)
    rk = v->Ref;
    vk = v->z2;

    lambda = v->lambda;
    gama = v->gama;

    ek = rk-yk;    // tracking error e(k)=r(k)-y(k)
    v->ek = ek;
    umax = v->umax;

    if (fabs(ek) <= v->yl)
    {
        uk = v->k1*sig(ek,gama/(2-gama)) - v->k3*sig(vk,gama);
    }
    else
    {
        fqe = sqrt(2 * v->b * v->alpha * umax * fabs(ek)) - v->vs;     // fp(e)
        if (ek<0)
           fqe = -fqe;
        uk = v->k2*(fqe-vk);
    }

    uk = uk - v->fd*v->z3/v->b;    // disturbance compensation


    if (uk > umax)     // saturation
        uk = umax;
    else if (uk < -umax)
        uk = -umax;


    v->Out = uk;   // output of control signal

    ep = yk - v->z1;

    v->z1 = v->z1 + v->Ts* (v->z2 + (v->L1/lambda)*fal(lambda*lambda*ep,0.7,1));
    v->z2 = v->z2 + v->Ts* (v->z3 + v->b*uk + v->L2*fal(lambda*lambda*ep,0.4,1));
    v->z3 = v->z3 + v->Ts* (v->L3*lambda)*fal(lambda*lambda*ep,0.1,1);//NESO


}

float32 fal(float32 e, float32 p, float32 delta)
{
    float32 out;
    if (fabs(e) > delta)
    {
        out = pow(fabs(e),p);
        if (e<0)
            out=-out;
    }
    else
    {
        out = e * pow(delta,p-1.0);
    }
    return out;
}


float32 sig(float32 a, float32 gama)
{
    float32 out;
    out= pow(fabs(a),gama);
    if (a<0)
        out=-out;
    return out;
}
