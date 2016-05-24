#include "Contrl.h"

uint8_t Motor_State,FOC_Flag,ShortI_Flag=0,ShortI_Counter;
int16_t hPhaseAOffset,hPhaseBOffset;	
int16_t SPOffset,VoltageOffset;
int16_t OMEGA_Old,Error_OMEGA;
uint16_t SB_FilterHe,SB_Temp;
// ADC 0: NC    1:current      2: SP   3:DC

LPF_16PARAMETERS SB_Filter_t,DC_Filter_t,MotorSpeed_t;
	
void Main_Loop(void)
{
	static uint16_t DelayStartT;
	
	if(T2ms_Flag)
	{		
		T2ms_Flag=0;
		ReadSignedADC0( &ReadADCParm );

		SB_Temp=SB_FilterHe;					//12bit
		
		if(SB_Temp>100) 		//
		{
			if((Motor_State!=RUN)||(DelayStartT>1000)) 
			{
				Motor_Start();
				DelayStartT=0;
				uGF.bit.MotorFail=0;
			}
			
			if(uGF.bit.MotorFail) 
			{
				DelayStartT++;	
			}		
		}	
		else if(SB_Temp>90) ;
		else 
		{
			if(Motor_State!=WAIT) Motor_Stop();

		}
	}
	

	if(T100ms_Flag)
	{
		T100ms_Flag=0;
		LED2Toggle();	
	}
}


void HardwareInit(void)
{
	GPIO_Iinitialization();
	TMER_Iinitialization();
	UART_Iinitialization();
	ADC_Iinitialization();
	Motor_Init();
	DAC_Iinitialization();
	
	SensorlessFOCinit();
}

void Motor_Init(void)
{
	Motor_State=INIT;
	SB_Filter_t.acc=0;
}

void Motor_Start(void)
{
	Motor_Stop();
	
	Open_PWM();
	Motor_State=RUN;
	ShortI_Flag=0;
	SetupParm();
// init Mode
	uGF.bit.ChangeSpeed = 0;
	uGF.bit.OpenLoop = 1;           // start in openloop
  uGF.bit.RunMotor = 1;           //then start motor
// Run the motor
  uGF.bit.ChangeMode = 1;	// Ensure variable initialization when open loop is	
	
	uGF.bit.CatchSpeed=1;				//EN catchSpeed
}

void Motor_Stop(void)
{
	Close_PWM();
	Motor_State=WAIT;
	
	uGF.bit.ChangeSpeed = 0;
	
	// zero out i sums 
	PIParmD.qdSum = 0;
	PIParmQ.qdSum = 0;
	PIParmW.qdSum = 0;
	
	// Stop the motor
	uGF.bit.RunMotor = 0;
	// executed for the first time
}

	
	
