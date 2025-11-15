
/* =================================================================================
File name:      RCNS.H  (float32 version)                    
                    
Originator:	Servo Control Systems Group, FZU

Description: 
Header file containing constants, data type definitions, and function prototypes
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 1-25-2018	Version 2.0
------------------------------------------------------------------------------*/
#include "DSP28x_Project.h"
#ifndef __RCNS_H__
#define __RCNS_H__
typedef struct { 
	float32 Ref;       // 输入：参考输入Input: Reference input r(k)
	float32 Fdb;   	    // 输入：反馈输入Input: Feedback y(k)
	float32 uk;
	float32 r0;
	float32 ek;
	float32 vk;
	float32 dk;
	float32 margin;      // 参数：跟踪误差ek的边界值
	float32 b;               // 参数：二阶模型的参数b:  ddot(y)=b*u+a*dot(y)
	float32 a;		// 参数：二阶模型的参数a
	float32 Ts;  		// 控制周期
	float32 zeta;		// 参数：RCNS线性控制律的极点阻尼比
	float32 omega;		// 参数：RCNS线性控制律的极点自然频率
	float32 zeta0;		// 参数：观测器的极点阻尼比
	float32 omega0;          // 参数：观测器的极点自然频率
	float32 alpha;	    // 参数：非线性函数的参数
	float32 beta;
	float32 alpha0;
	float32 rho;
	float32 f1;              // 参数：线性控制增益
	float32 f2;
	float32 fn1;			// 参数：非线性控制增益
	float32 fn2;
	float32 fr;			// 参数：给定r的前馈增益
	float32 fd;			// 参数：扰动d的前馈补偿增益
	float32 fc;			// 参数：扰动d的前馈补偿折扣系数（1-->0）:  uk=uk-fd*(1-fc)*dk
	float32 A11; 		    // 参数：xc(k+1)=Ao*xc(k)+Bu*u+By*y
	float32 A12; 		    // 参数：
	float32 A21; 		    // 参数：
	float32 A22; 		    // 参数：
	float32 B11; 		    // 参数：
	float32 B12; 		    // 参数：
	float32 B21; 		    // 参数：
	float32 B22; 		    // 参数：
	float32 L1; 		        // 参数：观测器增益矩阵元素1 -- Hat(x2,d)=xc-L*y
	float32 L2; 		        // 参数：观测器增益矩阵元素2
	float32 xc1; 	    // 内部变量：观测器的状态变量1
	float32 xc2; 	    // 内部变量：观测器的状态变量2
	float32 umin;		// 参数：控制量的饱和下限值
	float32 umax;		// 参数：控制量的饱和上限值
	float32 Out;	            // 实际输出的控制量：控制量 u(k)
	int tick;	            // 控制周期 计数值
	int tp;	            // 峰值时间 计数值
	float32 dmax;	       // 最大位移量
	float32 y0;	            // 初始位置
	float32 zhat;	            // zeta估计值
	float32 what;	            // wn估计值
	float32 ahat;	            // a估计值
	float32 bhat;	            // b估计值

	float32 ae1;	            //
	float32 ae2;	            //
	float32 tick1;	            //
	float32 tick2;	            //

	float32 gama;	            // 滤波因子
	void  (*init)();	       // 函数指针  Pointer to initialization function
	void  (*calc)();	       // 函数指针  Pointer to calculation function
} RCNS;

typedef RCNS *RCNS_handle;
/*-----------------------------------------------------------------------------
RCNS 默认初始化   Default initalizer for the RCNS object.
-----------------------------------------------------------------------------*/                     
#define RCNS_DEFAULTS {0, 0, 0, 0, 0, \
0, 0, 0, 0, 0, \
0, 0, 0, 0, 0, \
0, 0, 0, 0, 0, \
0, 0, 0, 0, 0, \
0, 0, 0, 0, 0, \
0, 0, 0, 0, 0, \
0, 0, 0, 0, 0, \
0, 0, 0, 0, 0, \
0, 0, 0, 0,  \
0, 0, 0, 0, 1, \
(void (*)(Uint32))rcns_init,\
(void (*)(Uint32))rcns_calc }
             	          

/*------------------------------------------------------------------------------
rcns.c文件中的函数原型   Prototypes for the functions in rcns.c
------------------------------------------------------------------------------*/
void rcns_init(RCNS_handle);
void rcns_calc(RCNS_handle);
#endif 

