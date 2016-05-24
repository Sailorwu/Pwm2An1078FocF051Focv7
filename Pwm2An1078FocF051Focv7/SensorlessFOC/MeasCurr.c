#include "Meascurr.h"

tMeasCurrParm MeasCurrParm;
tReadADCParm ReadADCParm;	// Struct used to read ADC values.

void ReadSignedADC0( tReadADCParm* pParm )
{
	SB_FilterHe=SP_16LPF(&SB_Filter_t,ADC_Tab[SP_Channl],8);
	pParm->qADValue=SB_FilterHe<<3;
	
	//DCBUS=ADC_Tab[VDC_Channl]<<3;
}

void MeasCompCurr(tParkParm* Iab)			//2013.7.21 MARK
{
	long LSdata;
	Iab->qIa=2*(MeasCurrParm.Offseta-(int16_t)ADC_Tab[IA_Channl]*8);
	Iab->qIb=2*(MeasCurrParm.Offsetb-(int16_t)ADC_Tab[IB_Channl]*8);
}

void InitMeasCompCurr( short Offset_a, short Offset_b )
{
	MeasCurrParm.Offseta=Offset_a;
	MeasCurrParm.Offsetb=Offset_b;
}
