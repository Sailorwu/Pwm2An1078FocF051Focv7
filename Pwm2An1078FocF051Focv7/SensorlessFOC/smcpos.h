
#ifndef smcpos_H
#define smcpos_H

#include "system_define.h"

typedef struct	{  
		SFRAC16  Valpha;   		// Input: Stationary alfa-axis stator voltage 
		SFRAC16  Ealpha;   		// Variable: Stationary alfa-axis back EMF 
		SFRAC16  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
		SFRAC16  Zalpha;      	// Output: Stationary alfa-axis sliding control 
		SFRAC16  Gsmopos_a;    	// Parameter: Motor dependent control gain 
		SFRAC16  Gsmopos_b;    	// Parameter: Motor dependent control gain 	
		SFRAC16  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current 
		SFRAC16  Fsmopos_a;    	// Parameter: Motor dependent plant matrix 
		SFRAC16  Fsmopos_b;    	// Parameter: Motor dependent plant matrix 
		SFRAC16  Vbeta;   		// Input: Stationary beta-axis stator voltage 
		SFRAC16  Ebeta;  		// Variable: Stationary beta-axis back EMF 
		SFRAC16  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
		SFRAC16  Zbeta;      	// Output: Stationary beta-axis sliding control 
		SFRAC16  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current 
		SFRAC16  Ialpha;  		// Input: Stationary alfa-axis stator current 
		SFRAC16  IalphaError; 	// Variable: Stationary alfa-axis current error                 
		SFRAC16  Kslide;     	// Parameter: Sliding control gain 
		SFRAC16  MaxSMCError;  	// Parameter: Maximum current error for linear SMC 
		SFRAC16  Ibeta;  		// Input: Stationary beta-axis stator current 
		SFRAC16  IbetaError;  	// Variable: Stationary beta-axis current error                 
		SFRAC16  Kslf;       	// Parameter: Sliding control filter gain 
		SFRAC16  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
		SFRAC16  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
		SFRAC16  ThetaOffset;	// Output: Offset used to compensate rotor angle
		SFRAC16  Theta;			// Output: Compensated rotor angle 
		SFRAC16  Omega;     	// Output: Rotor speed
		SFRAC16  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
} SMC;	            

extern SMC smc1;

#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

// Define this in RPMs

#define SPEED0 MINSPEEDINRPM
#define SPEED1 NOMINALSPEEDINRPM

// Define this in Degrees, from 0 to 360

#define THETA_AT_ALL_SPEED 90

#define OMEGA0 				(float)(SPEED0 * LOOPTIMEINSEC * \
										IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGANOMINAL	(float)(NOMINALSPEEDINRPM * LOOPTIMEINSEC * \
                		IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
	
#define THETA_ALL (float)(THETA_AT_ALL_SPEED * 180.0 / 32768.0)

#define CONSTANT_PHASE_SHIFT Q15(THETA_ALL)

#define _PI 3.1416

extern SFRAC16 PrevTheta;
extern SFRAC16 AccumTheta;
extern WORD AccumThetaCnt;

void SMC_Position_Estimation (SMC* s);
void SMCInit(SMC* s);
void CalcEstI(SMC* s);
void CalcIError(SMC *s);
void CalcZalpha(SMC *s);
void CalcZbeta(SMC *s);
void CalcBEMF(SMC *s);
void CalcOmegaFltred(SMC *s);
SFRAC16 FracMpy(SFRAC16 mul_1, SFRAC16 mul_2);
SFRAC16 FracDiv(SFRAC16 num_1, SFRAC16 den_1);

#endif
		
