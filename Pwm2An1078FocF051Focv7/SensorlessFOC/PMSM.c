#define INITIALIZE
#include "system_define.h"


/********************* Variables to display data using DMCI *********************************/
int16_t count = 0; // delay for ramping the reference velocity 
int16_t VelReq = 0; 

SMC smc1 = SMC_DEFAULTS;

uint32_t Startup_Ramp = 0;	/* Start up ramp in open loop. This variable
								is incremented in CalculateParkAngle()
								subroutine, and it is assigned to 
								ParkParm.qAngle as follows:
								ParkParm.qAngle += (int)(Startup_Ramp >> 16);*/

uint16_t Startup_Lock = 0;	/* This is a counter that is incremented in
								CalculateParkAngle() every time it is called. 
								Once this counter has a value of LOCK_TIME, 
								then theta will start increasing moving the 
								motor in open loop. */
uint16_t CatchSpeed_Cnt=0;

uGFt uGF;

tPIParm     PIParmD;	// Structure definition for Flux component of current, or Id
tPIParm     PIParmQ;	// Structure definition for Torque component of current, or Iq
tPIParm     PIParmW;	// Structure definition for Speed, or Omega

// Speed Calculation Variables

WORD iADCisrCnt = 0;	// This Counter is used as a timeout for polling the push buttons
						// in main() subroutine. It will be reset to zero when it matches
						// dButPolLoopCnt defined in UserParms.h
SFRAC16 PrevTheta = 0;	// Previous theta which is then substracted from Theta to get
						// delta theta. This delta will be accumulated in AccumTheta, and
						// after a number of accumulations Omega is calculated.
SFRAC16 AccumTheta = 0;	// Accumulates delta theta over a number of times
WORD AccumThetaCnt = 0;	// Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is 
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.

// Vd and Vq vector limitation variables

int32_t qVdSquared = 0;	// This variable is used to know what is left from the VqVd vector
						// in order to have maximum output PWM without saturation. This is
						// done before executing Iq control loop at the end of DoControl()

SFRAC16 DCbus = 0;		// DC Bus measured continuously and stored in this variable
						// while motor is running. Will be compared with TargetDCbus
						// and Vd and Vq will be compensated depending on difference
						// between DCbus and TargetDCbus

SFRAC16 TargetDCbus = 0;// DC Bus is measured before running motor and stored in this
						// variable. Any variation on DC bus will be compared to this value
						// and compensated linearly.	

SFRAC16 Theta_error = 0;// This value is used to transition from open loop to closed looop. 
						// At the end of open loop ramp, there is a difference between 
						// forced angle and estimated angle. This difference is stored in 
						// Theta_error, and added to estimated theta (smc1.Theta) so the 
						// effective angle used for commutating the motor is the same at 
						// the end of open loop, and at the begining of closed loop. 
						// This Theta_error is then substracted from estimated theta 
						// gradually in increments of 0.05 degrees until the error is less
						// than 0.05 degrees.

/************* START OF MAIN FUNCTION ***************/
void SensorlessFOCinit(void)
{
	SMCInit(&smc1);
	SetupControlParameters(); 
	uGF.Word = 0;                   // clear flags

	#ifdef TORQUEMODE
    uGF.bit.EnTorqueMod = 1;
	#endif

	#ifdef ENVOLTRIPPLE
    uGF.bit.EnVoltRipCo = 1;
	#endif	
}

//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,

void DoControl( void )
{
  if( uGF.bit.OpenLoop )
  {
  // OPENLOOP:	force rotating angle, and control Iq and Id
	//				Also limits Vs vector to ensure maximum PWM duty
	//				cycle and no saturation

	// This If statement is executed only the first time we enter open loop,
	// everytime we run the motor
    if( uGF.bit.ChangeMode )
    {
        // just changed to openloop
        uGF.bit.ChangeMode = 0;
        // synchronize angles
        // VqRef & VdRef not used
        CtrlParm.qVqRef = 0;
        CtrlParm.qVdRef = 0;
				CtrlParm.qVelRef = 0;
				Startup_Lock = 0;
				CatchSpeed_Cnt=0;
				Startup_Ramp = 0;
				// Initialize SMC
				smc1.Valpha = 0;
				smc1.Ealpha = 0;
				smc1.EalphaFinal = 0;
				smc1.Zalpha = 0;
				smc1.EstIalpha = 0;
				smc1.Vbeta = 0;
				smc1.Ebeta = 0;
				smc1.EbetaFinal = 0;
				smc1.Zbeta = 0;
				smc1.EstIbeta = 0;
				smc1.Ialpha = 0;
				smc1.IalphaError = 0;
				smc1.Ibeta = 0;
				smc1.IbetaError = 0;
				smc1.Theta = 0;
				smc1.Omega = 0;
				
				PIParmW.qdSum=0;
				PIParmD.qdSum=0;
				PIParmQ.qdSum=0;
    }

		// Enter initial torque demand in Amps using REFINAMPS() macro.
		// Maximum Value for reference is defined by shunt resistor value and 
		// differential amplifier gain. Use this equation to calculate 
		// maximum torque in Amperes:
		// 
		// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
		//
		// For example:
		//
		// RSHUNT = 0.005
		// VDD = 3.3
		// DIFFAMPGAIN = 75
		//
		// Maximum torque reference in Amps is:
		//
		// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
		//
		// If motor requires more torque than Maximum torque to startup, user
		// needs to change either shunt resistors installed on the board,
		// or differential amplifier gain.
		if(uGF.bit.CatchSpeed)				//catchspeed tset
		{
			CtrlParm.qVqRef=0;
			CtrlParm.qVdRef=0;
		}
		else CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);
	
	  if(AccumThetaCnt == 0)
	  {
	    PIParmW.qInMeas = smc1.Omega;
		}
	
	  // PI control for D
	  PIParmD.qInMeas = ParkParm.qId;
	  PIParmD.qInRef  = CtrlParm.qVdRef;
	  CalcPI(&PIParmD);
	  ParkParm.qVd    = PIParmD.qOut;
	
		// Vector limitation
		// Vd is not limited
		// Vq is limited so the vector Vs is less than a maximum of 95%.
		// The 5% left is needed to be able to measure current through
		// shunt resistors.
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95
		// Vq = SQRT(0.95^2 - Vd^2)
		
		
		//qVdSquared = FracMpy(PIParmD.qOut, PIParmD.qOut);				MARK
		//PIParmQ.qOutMax = _Q15sqrt(Q15(0.95*0.95) - qVdSquared);	
		qVdSquared=PIParmD.qOut*PIParmD.qOut;				//int32_t  
		PIParmQ.qOutMax = _Q15sqrt( Q15(MAXDUTYE)*Q15(MAXDUTYE)- qVdSquared);		
		PIParmQ.qOutMin = -PIParmQ.qOutMax;
		
		// PI control for Q
		PIParmQ.qInMeas = ParkParm.qIq;
		PIParmQ.qInRef  = CtrlParm.qVqRef;
		CalcPI(&PIParmQ);
		ParkParm.qVq    = PIParmQ.qOut;
  }

  else
    // Closed Loop Vector Control
  {
		// Pressing one of the push buttons, speed reference (or torque reference
		// if enabled) will be doubled. This is done to test transient response
		// of the controllers
		if( ++count == SPEEDDELAY ) 
	  {
			VelReq = FracMpy(ReadADCParm.qADValue,Q15(OMEGANOMINAL-OMEGA0)) + Q15(OMEGA0);		
			
			if (CtrlParm.qVelRef <= VelReq) //normal speed
			{
			     CtrlParm.qVelRef += 1; 
			}
			else CtrlParm.qVelRef -= 1 ; 
	 		count = 0;
		}
		
		// When it first transition from open to closed loop, this If statement is
		// executed
    if( uGF.bit.ChangeMode )
    {
      // just changed from openloop
       uGF.bit.ChangeMode = 0;
			// An initial value is set for the speed controller accumulation.
			//
			// The first time the speed controller is executed, we want the output
			// to be the same as it was the last time open loop was executed. So,
			// last time open loop was executed, torque refefernce was constant,
			// and set to CtrlParm.qVqRef.
			//
			// First time in closed loop, CtrlParm.qVqRef = PIParmW.qdSum >> 16
			// assuming the error is zero at time zero. This is why we set 
			// PIParmW.qdSum = (long)CtrlParm.qVqRef << 16.
			PIParmW.qdSum = (long)CtrlParm.qVqRef << 14;
			Startup_Lock = 0;
			Startup_Ramp = 0;
				//velocity reference ramp begins at minimum speed
			CtrlParm.qVelRef = Q15(OMEGA0);
		
	  }  

    // Check to see if new velocity information is available by comparing
    // the number of interrupts per velocity calculation against the
    // number of velocity count samples taken.  If new velocity info
    // is available, calculate the new velocity value and execute
    // the speed control loop.

    if(AccumThetaCnt == 0)
    {
    	// Execute the velocity control loop
			PIParmW.qInMeas = smc1.Omega;
    	PIParmW.qInRef  = CtrlParm.qVelRef;
    	CalcPI(&PIParmW);
    	CtrlParm.qVqRef = PIParmW.qOut;
    }
     
    // If the application is running in torque mode, the velocity
    // control loop is bypassed.  The velocity reference value, read
    // from the potentiometer, is used directly as the torque 
    // reference, VqRef. This feature is enabled automatically only if
// #define TORQUEMODE is defined in UserParms.h. If this is not
		// defined, uGF.bit.EnTorqueMod bit can be set in debug mode to enable
		// torque mode as well.

		if (uGF.bit.EnTorqueMod)  CtrlParm.qVqRef = CtrlParm.qVelRef;

		// Get Id reference from Field Weakening table. If Field weakening
		// is not needed or user does not want to enable this feature, 
		// let NOMINALSPEEDINRPM be equal to FIELDWEAKSPEEDRPM in
		// UserParms.h
		CtrlParm.qVdRef = ID_REF;

    // PI control for D
    PIParmD.qInMeas = ParkParm.qId;
    PIParmD.qInRef  = CtrlParm.qVdRef;
    CalcPI(&PIParmD);

		// If voltage ripple compensation flag is set, adjust the output
		// of the D controller depending on measured DC Bus voltage. This 
		// feature is enabled automatically only if #define ENVOLTRIPPLE is 
		// defined in UserParms.h. If this is not defined, uGF.bit.EnVoltRipCo
		// bit can be set in debug mode to enable voltage ripple compensation.
		//
		// NOTE:
		//
		// If Input power supply has switching frequency noise, for example if a
		// switch mode power supply is used, Voltage Ripple Compensation is not
		// recommended, since it will generate spikes on Vd and Vq, which can
		// potentially make the controllers unstable.
		if(uGF.bit.EnVoltRipCo) ParkParm.qVd = VoltRippleComp(PIParmD.qOut);
		else ParkParm.qVd = PIParmD.qOut;

		// Vector limitation
		// Vd is not limited
		// Vq is limited so the vector Vs is less than a maximum of 95%. 
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95
		// Vq = SQRT(0.95^2 - Vd^2)
		//qVdSquared = FracMpy(ParkParm.qVd, ParkParm.qVd);				MARK
    //PIParmQ.qOutMax = _Q15sqrt(Q15(0.95*0.95) - qVdSquared);
		qVdSquared=ParkParm.qVd*ParkParm.qVd;				//int32_t  
		PIParmQ.qOutMax = _Q15sqrt( Q15(MAXDUTYE)*Q15(MAXDUTYE)- qVdSquared);		
		PIParmQ.qOutMin = -PIParmQ.qOutMax;

    // PI control for Q
    PIParmQ.qInMeas = ParkParm.qIq;
    PIParmQ.qInRef  = CtrlParm.qVqRef;
    CalcPI(&PIParmQ);

		// If voltage ripple compensation flag is set, adjust the output
		// of the Q controller depending on measured DC Bus voltage
		if(uGF.bit.EnVoltRipCo) ParkParm.qVq = VoltRippleComp(PIParmQ.qOut);
		else ParkParm.qVq = PIParmQ.qOut;

		// Limit, if motor is stalled, stop motor commutation
		if (smc1.OmegaFltred < 0)
		{
			uGF.bit.RunMotor = 0;
			uGF.bit.MotorFail	= 1;
    }
	}
}

//---------------------------------------------------------------------
// The ADC ISR does speed calculation and executes the vector update loop.
// The ADC sample and conversion is triggered by the PWM period.
// The speed calculation assumes a fixed time interval between calculations.
//---------------------------------------------------------------------

void SensorlessFOCRUN(void)
{
  iADCisrCnt++;
 
  if( uGF.bit.RunMotor )
  {     
		
    // Calculate qIa,qIb
    MeasCompCurr(&ParkParm);				//1us
    
    // Calculate commutation angle using estimator
    CalculateParkAngle();						//19us
		
    // Calculate qId,qIq from qSin,qCos,qIa,qIb
    ClarkePark(&ParkParm);					//2us
                   
    // Calculate control values
    DoControl();										//11us
		
    // Calculate qSin,qCos from qAngle
    SinCos(&ParkParm);							//1.5us
		
    // Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
    InvPark(&ParkParm);  					//3.5us  		
    // Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
    // Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
    CalcSVGen(&ParkParm); 				//4us
  } else Close_PWM();   
	return;
}

//---------------------------------------------------------------------
bool SetupParm(void)
{
// ============= Open Loop ======================
	// Motor End Speed Calculation
	// MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	// Then, * 65536 which is a right shift done in "void CalculateParkAngle(void)"
	// ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	MotorParm.LockTime = LOCKTIME;

// ============= ADC - Measure Current & Pot ======================
  // Scaling constants: Determined by calibration or hardware design.
   
  MeasCurrParm.qKa    = DQKA;    
  MeasCurrParm.qKb    = DQKB;   

	// Target DC Bus, without sign.
	TargetDCbus = ((SFRAC16)VoltageOffset) ;//+ Q15(0.5);
   
  return False;
}

void CalculateParkAngle(void)
{
	smc1.Ialpha = ParkParm.qIalpha;
	smc1.Ibeta = ParkParm.qIbeta;
	smc1.Valpha = ParkParm.qValpha;
	smc1.Vbeta = ParkParm.qVbeta;

	SMC_Position_Estimation(&smc1);

	if(uGF.bit.OpenLoop)	
	{
		if(uGF.bit.CatchSpeed)				//catchspeed
		{
			if(CatchSpeed_Cnt<CATCHSPEEDTIME) CatchSpeed_Cnt++;
			else
			{
				CatchSpeed_Cnt=0;
				uGF.bit.CatchSpeed=0;
						
 				if(smc1.Omega>CATCHSPEEDMIN)	
				{
					uGF.bit.MotorStatic=1;
					// This section enables closed loop, right after open loop ramp.
					uGF.bit.ChangeMode = 1;
					uGF.bit.OpenLoop = 0;
					Theta_error=0;
					CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);
				}
 				else if(smc1.Omega<-CATCHSPEEDMIN) 
				{
					uGF.bit.MotorStatic=2;
				}
 				else 
				{
					uGF.bit.MotorStatic=0;	
				}		
			}
		}
		else 
		{
			if (Startup_Lock < MotorParm.LockTime)
				Startup_Lock += 1;	// This variable is incremented until
									// lock time expires, them the open loop
									// ramp begins
			else if (Startup_Ramp < MotorParm.EndSpeed)
				// Ramp starts, and increases linearly until EndSpeed is reached.
				// After ramp, estimated theta is used to commutate motor.
				Startup_Ramp += DELTA_STARTUP_RAMP;
			else
			{
				// This section enables closed loop, right after open loop ramp.
				uGF.bit.ChangeMode = 1;
				uGF.bit.OpenLoop = 0;
				// Difference between force angle and estimated theta is saved,
				// so a soft transition is made when entering closed loop.
				Theta_error = ParkParm.qAngle - smc1.Theta;
			}
			ParkParm.qAngle += (int16_t)(Startup_Ramp >> 16);			
		}
	}
	else
	{
		// This value is used to transition from open loop to closed looop. 
		// At the end of open loop ramp, there is a difference between 
		// forced angle and estimated angle. This difference is stored in 
		// Theta_error, and added to estimated theta (smc1.Theta) so the 
		// effective angle used for commutating the motor is the same at 
		// the end of open loop, and at the begining of closed loop. 
		// This Theta_error is then substracted from estimated theta 
		// gradually in increments of 0.05 degrees until the error is less
		// than 0.05 degrees.
		ParkParm.qAngle = smc1.Theta + Theta_error;
		if (_Q15abs(Theta_error) > _0_05DEG)
		{
			if (Theta_error < 0)
				Theta_error += _0_05DEG;
			else
				Theta_error -= _0_05DEG;
		}
	}
	return;
}

void SetupControlParameters(void)
{

// ============= PI D Term ===============      
    PIParmD.qKp = DKP;       
    PIParmD.qKi = DKI;              
    PIParmD.qKc = DKC;       
    PIParmD.qOutMax = DOUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

// ============= PI Q Term ===============
    PIParmQ.qKp = QKP;    
    PIParmQ.qKi = QKI;
    PIParmQ.qKc = QKC;
    PIParmQ.qOutMax = QOUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    InitPI(&PIParmQ);

// ============= PI W Term ===============
    PIParmW.qKp = WKP;       
    PIParmW.qKi = WKI;       
    PIParmW.qKc = WKC;       
    PIParmW.qOutMax = WOUTMAX;   
    PIParmW.qOutMin = -PIParmW.qOutMax;

    InitPI(&PIParmW);
	return;
}

void DebounceDelay(void)
{
	long i;
	for (i = 0;i < 100000;i++)
		;
	return;
}

// NOTE:
//
// If Input power supply has switching frequency noise, for example if a
// switch mode power supply is used, Voltage Ripple Compensation is not
// recommended, since it will generate spikes on Vd and Vq, which can
// potentially make the controllers unstable.

SFRAC16 VoltRippleComp(SFRAC16 Vdq)
{
	SFRAC16 CompVdq;
	// DCbus is already updated with new DC Bus measurement
	// in ReadSignedADC0 subroutine.
	//
	// If target DC Bus is greater than what we measured last sample, adjust
	// output as follows:
	//
	//                  TargetDCbus - DCbus
	// CompVdq = Vdq + --------------------- * Vdq
	//                         DCbus
	//
	// If Measured DCbus is greater than target, then the following compensation
	// is implemented:
	//
	//            TargetDCbus 
	// CompVdq = ------------- * Vdq
	//               DCbus
	//
	// If target and measured are equal, no operation is made.
	//
	if (TargetDCbus > DCbus)
		CompVdq = Vdq + FracMpy(FracDiv(TargetDCbus - DCbus, DCbus), Vdq);
	else if (DCbus > TargetDCbus)
		CompVdq = FracMpy(FracDiv(TargetDCbus, DCbus), Vdq);
	else
		CompVdq = Vdq;

	return CompVdq;
}
