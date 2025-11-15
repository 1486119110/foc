/* =================================================================================
File name:      PTOS.H  (Float32 version)                    
Originator:	Control Systems Group, FZU
			
Description: 
Header file containing constants, data type definitions, and 
function prototypes for the Ptoc.
=====================================================================================
 History: 07-17-2012	Version 1.0
 History: 12-02-2013	Version 2.0
 History: 21-05-2015	Version 3.0  revised for integration
------------------------------------------------------------------------------*/
#include "DSP28x_Project.h"
#ifndef __PTOC_H__
#define __PTOC_H__
typedef struct {  
	float32  Ref;         // 输入：参考输入  Input: Reference input r(k)
	float32  Ref1;        // 输入： Previous Input: Reference input r(k-1)
	float32  Fdb;   	// 输入：反馈输入  Input: Feedback y(k)
	float32  b;			// 参数：双积分模型的参数b
	float32  a;            // 参数：模型的参数a
	float32  umax;	    // 参数：控制量的饱和限幅值
	float32  Ts;   	    // PTOC的控制周期
	float32  zeta;		// 参数：PTOC线性控制律的极点阻尼比
	float32  omega;		// 参数：PTOC线性控制律的极点自然频率
	float32  alpha; 		// 参数：PTOC 加速度折扣系数
	float32  zeta0;          // 参数：观测器极点阻尼
	float32  omega0;	    // 参数：观测器带宽
	float32  yl;		    // 参数：
	float32  vs; 		    // 参数：
	float32  k1;			// PTOC 控制律参数
	float32  k2;			// 参数：
	float32  fd;      // 扰动补偿系数
	float32  fpe_sqrt; 	// 参数：=sqrt(2*alpha*b*umax)
	float32  A11; 		// 参数：观测器参数
	float32  A12; 		// 参数：
	float32  A21; 		// 参数：
	float32  A22; 		// 参数：
	float32  B11; 		// 参数：
	float32  B12; 		// 参数：
	float32  B21; 		// 参数：
	float32  B22; 		// 参数：
	float32  L1; 		    // 参数：
	float32  L2; 		    // 参数：
	float32  xc1; 		// 内部变量：观测器的状态变量1
	float32  xc2; 		// 内部变量：观测器的状态变量2
	float32  vms;	    	// 参数：速度限值，可在线修改
	float32  em;          // 参数：切换控制律的减速段长度
	float32  vm;          // 参数：切换控制律的目标速度上限
	float32  kv;          // 参数：恒速控制的比例增益值
	float32  kc;          // 参数：恒速控制的积分增益值
	float32  ki;          // 参数：线性区积分控制的增益值
	float32  zk;          // 积分项
	float32  SatErr;
	int16  sts;           // 参数：切换控制律的状态标志
	float32  Out;		    // 输出：控制量 u(k)
	float32  vk;
	float32  dk;
	float32  ek;
	float32  speed;
	float32  cptos;       // cptos=1, or dptos
	float32  lam1;
	float32  lam2;
	float32  lam3;
	float32  xc3;
	void  (*init)();	    // 函数指针  Pointer to initialization function
	void  (*calc)();	    // 函数指针  Pointer to calculation function
} PTOC;

typedef PTOC *PTOC_handle;
/*-----------------------------------------------------------------------------
PTOC 默认初始化   Default initalizer for the PTOC object.
-----------------------------------------------------------------------------*/                     
#define PTOC_DEFAULTS {    0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, 0, 0, 0, 0, 1,0,0,0,0,\
		(void (*)(Uint32))ptoc_init, \
		(void (*)(Uint32))ptoc_calc }
             	          

/*------------------------------------------------------------------------------
PTOC.C文件中的函数原型   Prototypes for the functions in PTOC.C
------------------------------------------------------------------------------*/
void ptoc_init(PTOC_handle);
void ptoc_calc(PTOC_handle);
#endif 

