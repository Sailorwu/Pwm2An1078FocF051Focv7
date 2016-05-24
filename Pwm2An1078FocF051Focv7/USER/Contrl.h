#ifndef _CONTRL_H_
#define _CONTRL_H_

#include "system_define.h"

#define INIT 		0
#define START 	1
#define RUN 		2
#define BREAK 	3
#define STOP 		4
#define WAIT    5
#define DOWN		6

#define SPVMIN	100
#define SPVDTA	10
#define SPVMAX	600

typedef struct LPF_32Parameters
{
	int32_t acc;
	int32_t acc_pre;
}LPF_32PARAMETERS;

typedef struct LPF_16Parameters
{
	int16_t acc;
	int16_t acc_pre;
}LPF_16PARAMETERS;

extern int16_t DCbus;
extern uint8_t Motor_State,ShortI_Flag,ShortI_Counter;
extern int16_t hPhaseAOffset,hPhaseBOffset;
extern int16_t SPOffset,VoltageOffset;
extern int16_t OMEGA_Old,Error_OMEGA;
extern uint8_t FOC_Flag;
extern uint16_t SB_FilterHe,SB_Temp;
extern LPF_16PARAMETERS SB_Filter_t,DC_Filter_tMotorSpeed_t;


void HardwareInit(void);
void GPIO_Iinitialization(void);
void DAC_Iinitialization(void);
void Motor_Init(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_Run(void);
void Main_Loop(void);
void Motor_Break(void);
void ADCTemp_Init(uint16_t* ADCnum);
int32_t SP_32LPF(LPF_32PARAMETERS * p_LPF_input, int32_t x_in,uint8_t Z);
int16_t SP_16LPF(LPF_16PARAMETERS * p_LPF_input, int16_t x_in,uint8_t Z);
void SpeedCalculate(void);
#endif

