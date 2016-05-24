// This is a sample C program for Microsoft C/C++ 6.0.
// The generated DLL is to be linked to PSIM.

// To compile the program into DLL, you can open the workspace file "msvc_dll.dsw" 
// as provided.

// This sample program calculates the rms of a 60-Hz input in[0], and
// stores the output in out[0].

// Variables:
//      t: Time, passed from PSIM by value
//   delt: Time step, passed from PSIM by value
//     in: input array, passed from PSIM by reference
//    out: output array, sent back to PSIM (Note: the values of out[*] can
//         be modified in PSIM)

// The maximum length of the input and output array "in" and "out" is 30.

// Because we used static/global variables in this example, the DLL 
// can only be used once per schematic file.  

#include <math.h>

//Defines
typedef unsigned short WORD;
//typedef signed int SFRAC16;
typedef unsigned char  BYTE;
//typedef unsigned char  BOOL;
// Structs

#define False  0
#define True   1

#define MCPWM_DEAD_TIME_CYCLES			80		// Dead time
#define MCPWM_RPM_TIMER_FREQ			1000000.0	// Frequency of the RPM measurement timer
#define MCPWM_MIN_DUTY_CYCLE			0.005	// Minimum duty cycle
#define MCPWM_MAX_DUTY_CYCLE			0.95	// Maximum duty cycle
#define MCPWM_RAMP_STEP					0.01	// Ramping step (1000 times/sec) at maximum duty cycle
#define MCPWM_RAMP_STEP_CURRENT_MAX		0.04	// Maximum ramping step (1000 times/sec) for the current control
#define MCPWM_RAMP_STEP_RPM_LIMIT		0.0005	// Ramping step when limiting the RPM

#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

#define PLLIN		8000000		// External Crystal or Clock Frequency (Hz)
#define DESIREDMIPS	40000000	// Enter desired MIPS. If RTDM is used, copy
								// this value to RTDM_FCY in RTDMUSER.h file


//************** Start-Up Parameters **************

#define LOCKTIMEINSEC  0.25		// Initial rotor lock time in seconds
								// Make sure LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC)
								// is less than 65535.
#define OPENLOOPTIMEINSEC 5.0	// Open loop time in seconds. This is the time that
								// will take from stand still to closed loop.
								// Optimized to overcome the brake inertia.
								// (Magtrol AHB-3 brake inertia = 6.89 kg x cm2).
#define INITIALTORQUE	5		// Initial Torque demand in Amps.
								// Enter initial torque demand in Amps using REFINAMPS() 
								// macro. Maximum Value for reference is defined by 
								// shunt resistor value and differential amplifier gain.
								// Use this equation to calculate maximum torque in 
								// Amperes:
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
#define ENDSPEEDOPENLOOP MINSPEEDINRPM

//************** Motor Parameters **************

// Values used to test Bejing Eletechnic Motor "Dia 80-252140-220" at 320VDC input. Motor datasheet at www.eletechnic.com
#define POLEPAIRS      12      // Number of pole pairs
#define PHASERES		((float)4.3)	// Phase resistance in Ohms.
#define PHASEIND		((float)0.027)// Phase inductance in Henrys.
#define NOMINALSPEEDINRPM 1000	// Make sure NOMINALSPEEDINRPM generates a MAXOMEGA < 1.0
		//50Hz   				// Use this formula:
								// MAXOMEGA = NOMINALSPEEDINRPM*SPEEDLOOPTIME*POLEPAIRS*2/60
								// If MAXOMEGA > 1.0, reduce NOMINALSPEEDINRPM or execute
								// speed loop faster by reducing SpeedLoopTime.
								// Maximum position of POT will set a reference of 
								// NOMINALSPEEDINRPM.
#define MINSPEEDINRPM	10	// Minimum speed in RPM. Closed loop will operate at this
		//8.47Hz						// speed. Open loop will transition to closed loop at
								// this minimum speed. Minimum POT position (CCW) will set
								// a speed reference of MINSPEEDINRPM
#define FIELDWEAKSPEEDRPM 1000	// Make sure FIELDWEAKSPEEDRPM generates a MAXOMEGA < 1.0
								// Use this formula:
								// MAXOMEGA = FIELDWEAKSPEEDRPM*SPEEDLOOPTIME*POLEPAIRS*2/60
								// If MAXOMEGA > 1.0, reduce FIELDWEAKSPEEDRPM or execute
								// speed loop faster by reducing SpeedLoopTime.
								// Maximum position of POT will set a reference of 
								// FIELDWEAKSPEEDRPM.




//************** Oscillator Parameters **************

//#define PLLIN		8000000		// External Crystal or Clock Frequency (Hz)
//#define DESIREDMIPS	40000000	// Enter desired MIPS. If RTDM is used, copy
								// this value to RTDM_FCY in RTDMUSER.h file

//************** PWM and Control Timing Parameters **********

#define PWMFREQUENCY	8000		// PWM Frequency in Hertz
#define DEADTIMESEC		0.000002f	// Deadtime in seconds
#define BUTPOLLOOPTIME	0.100f		// Button polling loop period in sec
#define SPEEDLOOPFREQ	1000		// Speed loop Frequency in Hertz. This value must
									// be an integer to avoid pre-compiler error
									// Use this value to test low speed motor

//************** Slide Mode Controller Parameters **********

#define SMCGAIN			0.85f		// Slide Mode Controller Gain (0.0 to 0.9999)
#define MAXLINEARSMC    0.005f		// If measured current - estimated current
								// is less than MAXLINEARSMC, the slide mode
								// Controller will have a linear behavior
								// instead of ON/OFF. Value from (0.0 to 0.9999)
#define FILTERDELAY		90		// Phase delay of two low pass filters for
								// theta estimation. Value in Degrees from
								// from 0 to 359.

//************** Hardware Parameters ****************

#define RSHUNT			0.001f		// Value in Ohms of shunt resistors used.
#define DIFFAMPGAIN		10		// Gain of differential amplifier.
#define VDD				3.3f		// VDD voltage, only used to convert torque
								// reference from Amps to internal variables

#define SPEEDDELAY 0.01f // Delay for the speed ramp.
					  // Necessary for the PI control to work properly at high speeds.

//*************** Optional Modes **************
//#define TORQUEMODE
//#define ENVOLTRIPPLE

//************** PI Coefficients **************

//******** D Control Loop Coefficients *******
#define     DKP        1.0f
#define     DKI        0.000268f
#define     DKC        0.99999f
#define     DOUTMAX    0.99999f

//******** Q Control Loop Coefficients *******
#define     QKP        1.0f
#define     QKI        0.000268f
#define     QKC        0.99999f
#define     QOUTMAX    0.99999f

//*** Velocity Control Loop Coefficients *****
#define     WKP        0.12f
#define     WKI        0.01f
#define     WKC        0.99999f
#define     WOUTMAX    0.95f

//************** ADC Scaling **************
// Scaling constants: Determined by calibration or hardware design. 
#define     DQK        (OMEGA10 - OMEGA1)/2.0f	// POT Scaling
#define     DQKA       0.080566406f	// Current feedback software gain : adc*(1/resol)*(AVDD/AmpGAIN)*(1/R) 
#define     DQKB       0.080566406f	// Current feedback software gain : adc*(1/4096)*(3.3/10)*(1/0.001)

//************** Field Weakening **************
// Enter flux demand Amperes using REFINAMPS() macro. Maximum Value for
// reference is defined by shunt resistor value and differential amplifier gain.
// Use this equation to calculate maximum torque in Amperes:
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
// in order to have field weakening, this reference value should be negative,
// so maximum value in this example is -4.4, or REFINAMPS(-4.4)

//****Values for Field weakening used to test Sander Motor at 160VDC input****
// For other motors, FW was not used to reach the rated speed
#define     dqKFw0  REFINAMPS(0)
#define     dqKFw1  REFINAMPS(-0.320)
#define     dqKFw2  REFINAMPS(-0.325)
#define     dqKFw3  REFINAMPS(-0.330)
#define     dqKFw4  REFINAMPS(-0.335)
#define     dqKFw5  REFINAMPS(-0.340)
#define     dqKFw6  REFINAMPS(-0.345)
#define     dqKFw7  REFINAMPS(-0.350)
#define     dqKFw8  REFINAMPS(-0.355)
#define     dqKFw9  REFINAMPS(-0.360)
#define     dqKFw10  REFINAMPS(-0.365)
#define     dqKFw11  REFINAMPS(-0.370)
#define     dqKFw12  REFINAMPS(-0.375)
#define     dqKFw13  REFINAMPS(-0.380)
#define     dqKFw14  REFINAMPS(-0.385)
#define     dqKFw15  REFINAMPS(-0.390)

//************** Derived Parameters ****************


#define LOOPTIMEINSEC (float)(1.0/PWMFREQUENCY) // PWM Period = 1.0 / PWMFREQUENCY
#define IRP_PERCALC (float)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define LOOPINTCY	 1
#define LOCKTIME	(int)(LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC))
// Time it takes to ramp from zero to MINSPEEDINRPM. Time represented in seconds
#define DELTA_STARTUP_RAMP	(float)(MINSPEEDINRPM*POLEPAIRS*LOOPTIMEINSEC* LOOPTIMEINSEC/(60*OPENLOOPTIMEINSEC))
// Number of control loops that must execute before the button routine is executed.
#define	BUTPOLLOOPCNT	(float)(BUTPOLLOOPTIME/LOOPTIMEINSEC)

// This pre-processor condition will generate an error if maximum speed is out of
// range on Q15 when calculating Omega.
#if (FIELDWEAKSPEEDRPM < NOMINALSPEEDINRPM)
	#error FIELDWEAKSPEEDRPM must be greater than NOMINALSPEEDINRPM for field weakening.
	#error if application does not require Field Weakening, set FIELDWEAKSPEEDRPM value
	#error equal to NOMINALSPEEDINRPM
#else
	#if ((FIELDWEAKSPEEDRPM*POLEPAIRS*2/(60*SPEEDLOOPFREQ)) >= 1)
		#error FIELDWEAKSPEEDRPM will generate an Omega value greater than 1 which is the
		#error maximum in Q15 format. Reduce FIELDWEAKSPEEDRPM value, or increase speed
		#error control loop frequency, SPEEDLOOPFREQ
	#endif
#endif


// Define this in RPMs

#define SPEED0 MINSPEEDINRPM
#define SPEED1 (SPEED0 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED2 (SPEED1 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED3 (SPEED2 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED4 (SPEED3 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED5 (SPEED4 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED6 (SPEED5 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED7 (SPEED6 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED8 (SPEED7 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED9 (SPEED8 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED10 (FIELDWEAKSPEEDRPM)

// Define this in Degrees, from 0 to 360

#define THETA_AT_ALL_SPEED 90

#define OMEGA0 (float)(SPEED0 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA1 (float)(SPEED1 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA2 (float)(SPEED2 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA3 (float)(SPEED3 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA4 (float)(SPEED4 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA5 (float)(SPEED5 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA6 (float)(SPEED6 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA7 (float)(SPEED7 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA8 (float)(SPEED8 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA9 (float)(SPEED9 * LOOPTIMEINSEC *              IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA10 (float)(SPEED10 * LOOPTIMEINSEC *          IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)


#define OMEGANOMINAL	(float)(NOMINALSPEEDINRPM * LOOPTIMEINSEC *            		IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGAFIELDWK	(float)(FIELDWEAKSPEEDRPM * LOOPTIMEINSEC *               		IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)

#define THETA_ALL (float)(THETA_AT_ALL_SPEED * 180.0 / 32768.0)
#define CONSTANT_PHASE_SHIFT (THETA_ALL)

#define		PI				3.141592654f
#define		SQRT2			1.414213562f
#define		SQRT3			1.732050808f
#define		INV_SQRT3		(float)(1./SQRT3)

#define REFINAMPS(Amperes_Value) (float)(	Amperes_Value*DQKA*RSHUNT*DIFFAMPGAIN/(VDD/2))


union{
        struct {
		unsigned OpenLoop:1;	// Indicates if motor is running in open or closed loop
		unsigned RunMotor:1;	// If motor is running, or stopped.
		unsigned EnTorqueMod:1;	// This bit enables Torque mode when running closed loop
		unsigned EnVoltRipCo:1;	// Bit that enables Voltage Ripple Compensation
		unsigned Btn1Pressed:1;	// Button 1 has been pressed.
		unsigned ChangeMode:1;	// This flag indicates that a transition from open to closed
								// loop, or closed to open loop has happened. This
								// causes DoControl subroutine to initialize some variables
								// before executing open or closed loop for the first time
		unsigned ChangeSpeed:1;	// This flag indicates a step command in speed reference.
								// This is mainly used to analyze step response
		unsigned    :9;
            }bit;
        	WORD Word;
 } uGF;


struct {

	float LockTime;

	float EndSpeed;

} MotorParm;

typedef struct {
	float	qKa;	
	short	Offseta;

	float	qKb;   
	short	Offsetb;
} tMeasCurrParm;



typedef struct {
		float  Valpha;   		// Input: Stationary alfa-axis stator voltage
		float  Ealpha;   		// Variable: Stationary alfa-axis back EMF
		float  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
		float  Zalpha;      	// Output: Stationary alfa-axis sliding control
		float  Gsmopos;    	// Parameter: Motor dependent control gain
		float  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current
		float  Fsmopos;    	// Parameter: Motor dependent plant matrix
		float  Vbeta;   		// Input: Stationary beta-axis stator voltage
		float  Ebeta;  		// Variable: Stationary beta-axis back EMF
		float  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
		float  Zbeta;      	// Output: Stationary beta-axis sliding control
		float  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current
		float  Ialpha;  		// Input: Stationary alfa-axis stator current
		float  IalphaError; 	// Variable: Stationary alfa-axis current error
		float  Kslide;     	// Parameter: Sliding control gain
		float  MaxSMCError;  	// Parameter: Maximum current error for linear SMC
		float  Ibeta;  		// Input: Stationary beta-axis stator current
		float  IbetaError;  	// Variable: Stationary beta-axis current error
		float  Kslf;       	// Parameter: Sliding control filter gain
		float  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
		float  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
		float  ThetaOffset;	// Output: Offset used to compensate rotor angle
		float  Theta;			// Output: Compensated rotor angle
		float  Omega;     	// Output: Rotor speed
		float  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
} SMC;
typedef SMC *SMC_handle;
typedef struct {
    float    qdSum;          // 1.31 format
    float   qKp;
    float   qKi;
    float   qKc;
    float   qOutMax;
    float   qOutMin;
    float   qInRef; 
    float   qInMeas;
    float   qOut;
    } tPIParm;
typedef struct {
    float   qAngle;
    float   qSin;
    float   qCos;
    float   qIa;
    float   qIb;
    float   qIalpha;
    float   qIbeta;
    float   qId;
    float   qIq;
    float   qVd;
    float   qVq;
    float   qValpha;
    float   qVbeta;
    float   qV1;
    float   qV2;
    float   qV3;
    } tParkParm;
typedef struct {
    float   qVelRef;    // Reference velocity
    float   qVdRef;     // Vd flux reference value
    float   qVqRef;     // Vq torque reference value
    } tCtrlParm;

//------------------  C API for FdWeak routine ---------------------
typedef struct {
	float	qK1;            // < Nominal speed value
	float	qIdRef;
	float	qFwOnSpeed;
	float	qFwActiv;
	int	qIndex;
	float	qFWPercentage;
	float	qInterpolPortion;
	float		qFwCurve[16];	// Curve for magnetizing current
    } tFdWeakParm;
//------------------  C API for SVGen routine ---------------------
typedef struct {
	unsigned int   iPWMPeriod;

	float   qVr1;
	float   qVr2;
	float   qVr3;

	float T1;
	float T2;

	float Ta;
	float Tb;
	float Tc;

    } tSVGenParm;


// Private variables
int count = 0; // delay for ramping the reference velocity 
float VelReq = 0; 

float Startup_Ramp = 0;	/* Start up ramp in open loop. This variable
								is incremented in CalculateParkAngle()
								subroutine, and it is assigned to 
								ParkParm.qAngle as follows:
								ParkParm.qAngle += (int)(Startup_Ramp >> 16);*/

float Startup_Lock = 0;	/* This is a counter that is incremented in
								CalculateParkAngle() every time it is called. 
								Once this counter has a value of LOCK_TIME, 
								then theta will start increasing moving the 
								motor in open loop. */

static volatile tMeasCurrParm MeasCurrParm;
SMC smc1 = SMC_DEFAULTS;

tParkParm ParkParm;

tPIParm     PIParmD;	// Structure definition for Flux component of current, or Id
tPIParm     PIParmQ;	// Structure definition for Torque component of current, or Iq
tPIParm     PIParmW;	// Structure definition for Speed, or Omega

tCtrlParm CtrlParm;

tSVGenParm SVGenParm;

tFdWeakParm FdWeakParm;

int SpeedReference = 10;

unsigned int  switching_frequency_now = PWMFREQUENCY;

double OUT[3];
double IN[12];





// Speed Calculation Variables

WORD iADCisrCnt = 0;	// This Counter is used as a timeout for polling the push buttons
						// in main() subroutine. It will be reset to zero when it matches
						// dButPolLoopCnt defined in UserParms.h
float PrevTheta = 0;	// Previous theta which is then substracted from Theta to get
						// delta theta. This delta will be accumulated in AccumTheta, and
						// after a number of accumulations Omega is calculated.
float AccumTheta = 0;	// Accumulates delta theta over a number of times
WORD AccumThetaCnt = 0;	// Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is 
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.

// Vd and Vq vector limitation variables

static volatile float qVdSquared = 0;	// This variable is used to know what is left from the VqVd vector
						// in order to have maximum output PWM without saturation. This is
						// done before executing Iq control loop at the end of DoControl()
static volatile float DCbus = 0;		// DC Bus measured continuously and stored in this variable
						// while motor is running. Will be compared with TargetDCbus
						// and Vd and Vq will be compensated depending on difference
						// between DCbus and TargetDCbus
static volatile float TargetDCbus = 0;// DC Bus is measured before running motor and stored in this
						// variable. Any variation on DC bus will be compared to this value
						// and compensated linearly.	
static volatile float Theta_error = 0;// This value is used to transition from open loop to closed looop. 
						// At the end of open loop ramp, there is a difference between 
						// forced angle and estimated angle. This difference is stored in 
						// Theta_error, and added to estimated theta (smc1.Theta) so the 
						// effective angle used for commutating the motor is the same at 
						// the end of open loop, and at the begining of closed loop. 
						// This Theta_error is then substracted from estimated theta 
						// gradually in increments of 0.05 degrees until the error is less
						// than 0.05 degrees.

//===================================================
// extern function
void InitPI( tPIParm *pParm);




bool SetupParm(void)
{

// ============= Open Loop ======================
	// Motor End Speed Calculation
	// MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	// Then, * 65536 which is a right shift done in "void CalculateParkAngle(void)"
	// ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	MotorParm.EndSpeed = (float)ENDSPEEDOPENLOOP * (float)POLEPAIRS * LOOPTIMEINSEC / (float)60.0;
	MotorParm.LockTime = LOCKTIME;

// ============= ADC - Measure Current & Pot ======================

    MeasCurrParm.qKa    = DQKA;    
    MeasCurrParm.qKb    = DQKB;   


// ============= SVGen ===============
    // Set PWM period to Loop Time 
    SVGenParm.iPWMPeriod = LOOPINTCY;      

    
    return False;
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

void InitPI( tPIParm *pParm)
{
	pParm->qdSum=0;
	pParm->qOut=0;

	//pParm->qInMeas=0;
	//pParm->qInRef=0;
	//pParm->qKc=0;
	//pParm->qKi=0;
	//pParm->qKp=0;
	//pParm->qOutMax=0;
	//pParm->qOutMin=0;
}
void CalcPI( tPIParm *pParm)
{
	float U,Exc,Err;
	Err  = pParm->qInRef - pParm->qInMeas;
	
	U  = pParm->qdSum + pParm->qKp * Err;

	if( U > pParm->qOutMax )          pParm->qOut = pParm->qOutMax;
	else if( U < pParm->qOutMin )    pParm->qOut = pParm->qOutMin;
	else                  pParm->qOut = U ;

	Exc = U - pParm->qOut;

	pParm->qdSum = pParm->qdSum + pParm->qKi * Err - pParm->qKc * Exc ;
	
	return;
}
float VoltRippleComp(float Vdq)
{
	float CompVdq;
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
		CompVdq = Vdq + (((TargetDCbus - DCbus)/ DCbus)* Vdq);
	else if (DCbus > TargetDCbus)
		CompVdq = ((TargetDCbus/ DCbus)* Vdq);
	else
		CompVdq = Vdq;

	return CompVdq;
}
void SMCInit(SMC *s)
{
	//				  R * Ts
	// Fsmopos = 1 - --------
	//					L
	//			  Ts
	// Gsmopos = ----
	//			  L
	// Ts = Sampling Period. If sampling at PWM, Ts = 50 us
	// R = Phase Resistance. If not provided by motor datasheet,
	//	   measure phase to phase resistance with multimeter, and
	//	   divide over two to get phase resistance. If 4 Ohms are
	//	   measured from phase to phase, then R = 2 Ohms
	// L = Phase inductance. If not provided by motor datasheet,
	//	   measure phase to phase inductance with multimeter, and
	//	   divide over two to get phase inductance. If 2 mH are
	//	   measured from phase to phase, then L = 1 mH

	if ((PHASERES * LOOPTIMEINSEC) > PHASEIND)
		s->Fsmopos = 0.0f;
	else
		s->Fsmopos = (float)(1 - PHASERES * LOOPTIMEINSEC / PHASEIND);

	if (LOOPTIMEINSEC > PHASEIND)
		s->Gsmopos = 0.99999f;
	else
		s->Gsmopos = LOOPTIMEINSEC / PHASEIND;

	s->Kslide = SMCGAIN;
	s->MaxSMCError = MAXLINEARSMC;
	s->FiltOmCoef = (OMEGA0 * PI / IRP_PERCALC); // Cutoff frequency for omega filter
											 // is minimum omega, or OMEGA0
	return;
}	
void SMC_Position_Estimation (SMC *s)
{
	// Sliding mode current observer
	
	s->EstIalpha = s->Gsmopos * s->Valpha - s->Gsmopos * s->Ealpha	- s->Gsmopos * s->Zalpha	 + s->Fsmopos * s->EstIalpha;
	s->EstIbeta = s->Gsmopos * s->Vbeta - s->Gsmopos * s->Ebeta - s->Gsmopos * s->Zbeta + s->Fsmopos * s->EstIbeta;

	s->IalphaError = s->EstIalpha - s->Ialpha;
	s->IbetaError = s->EstIbeta - s->Ibeta;
		
	// Sliding control calculator

	if (fabsf(s->IalphaError) < s->MaxSMCError)
	{
		// s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zalpha will be proportional to the
		// error (Ialpha - EstIalpha) and slide mode gain, Kslide.
		s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError;
	}
	else if (s->IalphaError > 0)
		s->Zalpha = s->Kslide;
	else
		s->Zalpha = -s->Kslide;

	if (fabsf(s->IbetaError) < s->MaxSMCError)
	{
		// s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zbeta will be proportional to the
		// error (Ibeta - EstIbeta) and slide mode gain, Kslide.
		s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError;
	}
	else if (s->IbetaError > 0)
		s->Zbeta = s->Kslide;
	else
		s->Zbeta = -s->Kslide;
	
		
	// Sliding control filter -> back EMF calculator
	 s->Ealpha = s->Ealpha + s->Kslf * s->Zalpha - s->Kslf * s->Ealpha;
	 s->Ebeta = s->Ebeta + s->Kslf * s->Zbeta - s->Kslf * s->Ebeta;
	 s->EalphaFinal = s->EalphaFinal + s->KslfFinal * s->Ealpha  - s->KslfFinal * s->EalphaFinal;
	 s->EbetaFinal = s->EbetaFinal + s->KslfFinal * s->Ebeta - s->KslfFinal * s->EbetaFinal;
	
		
	// Rotor angle calculator -> Theta = atan(-EalphaFinal,EbetaFinal)

	s->Theta = atan2f(-s->EalphaFinal,s->EbetaFinal);
		
	AccumTheta += s->Theta - PrevTheta;
	PrevTheta = s->Theta;
	
	AccumThetaCnt++;
	if (AccumThetaCnt == IRP_PERCALC)
	{
		s->Omega = AccumTheta;
		AccumThetaCnt = 0;
		AccumTheta = 0;
	}
	//					  Q15(Omega) * 60
	// Speed RPMs = -----------------------------
	//				 SpeedLoopTime * Motor Poles
	// For example:
	//	  Omega = 0.5
	//	  SpeedLoopTime = 0.001
	//	  Motor Poles (pole pairs * 2) = 10
	// Then:
	//	  Speed in RPMs is 3,000 RPMs

	// s->OmegaFltred = s->OmegaFltred + FilterCoeff * s->Omega   - FilterCoeff * s->OmegaFltred

	 s->OmegaFltred = s->OmegaFltred + s->FiltOmCoef * s->Omega - s->FiltOmCoef * s->OmegaFltred;
		

	// Adaptive filter coefficients calculation
	// Cutoff frequency is defined as 2*_PI*electrical RPS
	//
	//		Wc = 2*_PI*Fc.
	//		Kslf = Tpwm*2*_PI*Fc
	//
	// Fc is the cutoff frequency of our filter. We want the cutoff frequency
	// be the frequency of the drive currents and voltages of the motor, which
	// is the electrical revolutions per second, or eRPS.
	//
	//		Fc = eRPS = RPM * Pole Pairs / 60
	//
	// Kslf is then calculated based on user parameters as follows:
	// First of all, we have the following equation for RPMS:
	//
	//		RPM = (Q15(Omega) * 60) / (SpeedLoopTime * Motor Poles)
	//		Let us use: Motor Poles = Pole Pairs * 2
	//		eRPS = RPM * Pole Pairs / 60), or
	//		eRPS = (Q15(Omega) * 60 * Pole Pairs) / (SpeedLoopTime * Pole Pairs * 2 * 60)
	//	Simplifying eRPS
	//		eRPS = Q15(Omega) / (SpeedLoopTime * 2)
	//	Using this equation to calculate Kslf
	//		Kslf = Tpwm*2*_PI*Q15(Omega) / (SpeedLoopTime * 2)
	//	Using diIrpPerCalc = SpeedLoopTime / Tpwm
	//		Kslf = Tpwm*2*Q15(Omega)*_PI / (diIrpPerCalc * Tpwm * 2)
	//	Simplifying:
	//		Kslf = Q15(Omega)*_PI/diIrpPerCalc
	//
	// We use a second filter to get a cleaner signal, with the same coefficient
	//
	//		Kslf = KslfFinal = Q15(Omega)*_PI/diIrpPerCalc
	//
	// What this allows us at the end is a fixed phase delay for theta compensation
	// in all speed range, since cutoff frequency is changing as the motor speeds up.
	// 
	// Phase delay: Since cutoff frequency is the same as the input frequency, we can
	// define phase delay as being constant of -45 DEG per filter. This is because
	// the equation to calculate phase shift of this low pass filter is 
	// arctan(Fin/Fc), and Fin/Fc = 1 since they are equal, hence arctan(1) = 45 DEG.
	// A total of -90 DEG after the two filters implemented (Kslf and KslfFinal).
	
	s->Kslf = s->KslfFinal = s->OmegaFltred * (PI / IRP_PERCALC);
		
	// Since filter coefficients are dynamic, we need to make sure we have a minimum
	// so we define the lowest operation speed as the lowest filter coefficient

	if (s->Kslf < (OMEGA0 * PI / IRP_PERCALC))
	{
		s->Kslf = (OMEGA0 * PI / IRP_PERCALC);
		s->KslfFinal = (OMEGA0 * PI / IRP_PERCALC);
	}
	s->ThetaOffset = CONSTANT_PHASE_SHIFT;
	s->Theta = s->Theta + s->ThetaOffset;

	return;
}
float FieldWeakening(float qMotorSpeed)
{
    /* if the speed is less than one for activating the FW */
	if (qMotorSpeed <= FdWeakParm.qFwOnSpeed)
	{
		/* set Idref as first value in magnetizing curve */
		FdWeakParm.qIdRef = FdWeakParm.qFwCurve[0];
	} 
	else
	{
		// Index in FW-Table. The result is left shifted 11 times because
		// we have a field weakening table of 16 (4 bits) values, and the result
		// of the division is 15 bits (16 bits, with no sign). So
		// Result (15 bits) >> 11 -> Index (4 bits).
		FdWeakParm.qFWPercentage = (float)(qMotorSpeed-FdWeakParm.qFwOnSpeed)/ (OMEGAFIELDWK-OMEGANOMINAL+1);
		FdWeakParm.qIndex = (int)(FdWeakParm.qFWPercentage * 32);

		// Interpolation betwen two results from the Table. First mask 11 bits,
		// then left shift 4 times to get 15 bits again.
		FdWeakParm.qInterpolPortion = (float)((int)(FdWeakParm.qFWPercentage*32768) & 0x07FF) /2048;

		FdWeakParm.qIdRef = FdWeakParm.qFwCurve[FdWeakParm.qIndex] - ((FdWeakParm.qFwCurve[FdWeakParm.qIndex] - FdWeakParm.qFwCurve[FdWeakParm.qIndex+1] ) /FdWeakParm.qInterpolPortion);

	}

	return FdWeakParm.qIdRef;
}
void FWInit(void)
{
	/* initialize magnetizing curve values */
	FdWeakParm.qFwOnSpeed = OMEGANOMINAL;
	FdWeakParm.qFwCurve[0]	= dqKFw0;
	FdWeakParm.qFwCurve[1]	= dqKFw1;
	FdWeakParm.qFwCurve[2]	= dqKFw2;
	FdWeakParm.qFwCurve[3]	= dqKFw3;
	FdWeakParm.qFwCurve[4]	= dqKFw4;
	FdWeakParm.qFwCurve[5]	= dqKFw5;
	FdWeakParm.qFwCurve[6]	= dqKFw6;
	FdWeakParm.qFwCurve[7]	= dqKFw7;
	FdWeakParm.qFwCurve[8]	= dqKFw8;
	FdWeakParm.qFwCurve[9]	= dqKFw9;
	FdWeakParm.qFwCurve[10]	= dqKFw10;
	FdWeakParm.qFwCurve[11]	= dqKFw11;
	FdWeakParm.qFwCurve[12]	= dqKFw12;
	FdWeakParm.qFwCurve[13]	= dqKFw13;
	FdWeakParm.qFwCurve[14]	= dqKFw14;
	FdWeakParm.qFwCurve[15]	= dqKFw15;	
	return;
}



void ClarkePark(void)
{
	ParkParm.qIalpha = ParkParm.qIa;
	ParkParm.qIbeta = ParkParm.qIa*INV_SQRT3 + 2*ParkParm.qIb*INV_SQRT3;
	// Ialpha and Ibeta have been calculated. Now do rotation.
	// Get qSin, qCos from ParkParm structure

	ParkParm.qId =  ParkParm.qIalpha*cosf(ParkParm.qAngle) + ParkParm.qIbeta*sinf(ParkParm.qAngle);
	ParkParm.qIq = -ParkParm.qIalpha*sinf(ParkParm.qAngle) + ParkParm.qIbeta*cosf(ParkParm.qAngle);

	return;
}


static double testpoint = 0.0f;

void DoControl( void )
	{


		AccumThetaCnt++;
		if (AccumThetaCnt == IRP_PERCALC)
		{
			AccumThetaCnt = 0;
		}


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
	
			CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);
	
			if(AccumThetaCnt == 0)
			{
				PIParmW.qInMeas = smc1.Omega;
			}
	
			// PI control for D
			PIParmD.qInMeas = ParkParm.qId;
			PIParmD.qInRef	= CtrlParm.qVdRef;
			CalcPI(&PIParmD);
			ParkParm.qVd	= PIParmD.qOut;
	
			// Vector limitation
			// Vd is not limited
			// Vq is limited so the vector Vs is less than a maximum of 95%.
			// The 5% left is needed to be able to measure current through
			// shunt resistors.
			// Vs = SQRT(Vd^2 + Vq^2) < 0.95
			// Vq = SQRT(0.95^2 - Vd^2)
			qVdSquared = PIParmD.qOut * PIParmD.qOut;
			PIParmQ.qOutMax = sqrtf((0.95f*0.95f) - qVdSquared);
			PIParmQ.qOutMin = -PIParmQ.qOutMax;
	
			// PI control for Q
			PIParmQ.qInMeas = ParkParm.qIq;
			PIParmQ.qInRef	= CtrlParm.qVqRef;
			CalcPI(&PIParmQ);
			ParkParm.qVq	= PIParmQ.qOut;
			
		}
	
		else
		// Closed Loop Vector Control
		{

			// Pressing one of the push buttons, speed reference (or torque reference
			// if enabled) will be doubled. This is done to test transient response
			// of the controllers
			if( ++count == SPEEDDELAY ) 
			{
				VelReq = ((float)SpeedReference* DQK) + ((OMEGA10 + OMEGA1)/2.0f);
	
				if (CtrlParm.qVelRef <= VelReq)
				{
					 CtrlParm.qVelRef += SPEEDDELAY; 
				}
				else 
				{
					CtrlParm.qVelRef -= SPEEDDELAY;
				}
			
				count = 0;
			}

			CtrlParm.qVelRef = (float)SpeedReference  * LOOPTIMEINSEC * IRP_PERCALC * POLEPAIRS * 2.0f / 60.0f;
			
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
				PIParmW.qdSum = CtrlParm.qVqRef ;
				Startup_Lock = 0;
				Startup_Ramp = 0;
					//velocity reference ramp begins at minimum speed
				CtrlParm.qVelRef = OMEGA0;
			
			}  
	
			// Check to see if new velocity information is available by comparing
			// the number of interrupts per velocity calculation against the
			// number of velocity count samples taken.	If new velocity info
			// is available, calculate the new velocity value and execute
			// the speed control loop.
	
			if(AccumThetaCnt == 0)
			{
				// Execute the velocity control loop
				PIParmW.qInMeas = smc1.Omega;
				PIParmW.qInRef	= CtrlParm.qVelRef;
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
	
			if (uGF.bit.EnTorqueMod)
				CtrlParm.qVqRef = CtrlParm.qVelRef;
	
			// Get Id reference from Field Weakening table. If Field weakening
			// is not needed or user does not want to enable this feature, 
			// let NOMINALSPEEDINRPM be equal to FIELDWEAKSPEEDRPM in
			// UserParms.h
			CtrlParm.qVdRef = FieldWeakening(fabsf(CtrlParm.qVelRef));

			CtrlParm.qVdRef = 10.0f;

			// PI control for D
			PIParmD.qInMeas = ParkParm.qId;
			PIParmD.qInRef	= CtrlParm.qVdRef;
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
			if(uGF.bit.EnVoltRipCo)
				ParkParm.qVd = VoltRippleComp(PIParmD.qOut);
			else
				ParkParm.qVd = PIParmD.qOut;
	
			// Vector limitation
			// Vd is not limited
			// Vq is limited so the vector Vs is less than a maximum of 95%. 
			// Vs = SQRT(Vd^2 + Vq^2) < 0.95
			// Vq = SQRT(0.95^2 - Vd^2)
			qVdSquared = ParkParm.qVd * ParkParm.qVd;
			PIParmQ.qOutMax = sqrtf((0.95f*0.95f) - qVdSquared);
			PIParmQ.qOutMin = -PIParmQ.qOutMax;
	
			// PI control for Q
			PIParmQ.qInMeas = ParkParm.qIq;
			//CtrlParm.qVqRef = 0.5f;
			PIParmQ.qInRef	= CtrlParm.qVqRef;
			CalcPI(&PIParmQ);
	
			// If voltage ripple compensation flag is set, adjust the output
			// of the Q controller depending on measured DC Bus voltage
			if(uGF.bit.EnVoltRipCo)
				ParkParm.qVq = VoltRippleComp(PIParmQ.qOut);
			else
				ParkParm.qVq = PIParmQ.qOut;
	
			// Limit, if motor is stalled, stop motor commutation
			if (smc1.OmegaFltred < 0)
			{
				//=================uGF.bit.RunMotor = 0;
			}
		}
	}
void MeasCompCurr( void )
{
	 //int CorrADC1, CorrADC2;

	 //int curr1 = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
	 //int curr2 = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
	 
	 //CorrADC1 = curr1 - MeasCurrParm.Offseta;
	 //CorrADC2 = curr2 - MeasCurrParm.Offsetb;

	 //ParkParm.qIa = MeasCurrParm.qKa * (float)CorrADC1;
	 //ParkParm.qIb = MeasCurrParm.qKb * (float)CorrADC2;

	 ParkParm.qIa =(float)IN[0];
	 ParkParm.qIb =(float)IN[1];

	 return;
}


void CalculateParkAngle(void)
{
	smc1.Ialpha = ParkParm.qIalpha;
	smc1.Ibeta = ParkParm.qIbeta;
	smc1.Valpha = ParkParm.qValpha;
	smc1.Vbeta = ParkParm.qVbeta;

	SMC_Position_Estimation(&smc1);


	if(uGF.bit.OpenLoop ==1)	
	{
		if (Startup_Lock < MotorParm.LockTime)
		{
			Startup_Lock += 0.003f;	// This variable is incremented until
								// lock time expires, them the open loop
								// ramp begins
		}
		else if (Startup_Ramp < MotorParm.EndSpeed)
		{
			// Ramp starts, and increases linearly until EndSpeed is reached.
			// After ramp, estimated theta is used to commutate motor.
			Startup_Ramp += DELTA_STARTUP_RAMP;
		}
		else
		{
			// This section enables closed loop, right after open loop ramp.
			uGF.bit.ChangeMode = 1;
			uGF.bit.OpenLoop = 0;
			// Difference between force angle and estimated theta is saved,
			// so a soft transition is made when entering closed loop.
			Theta_error = ParkParm.qAngle - smc1.Theta;
		}
		ParkParm.qAngle += Startup_Ramp;


	}
	else if(uGF.bit.OpenLoop ==0)
	{

		// This value is used to transition from open loop to closed looop. 
		// At the end of open loop ramp, there is a difference between 
		// forced angle and estimated angle. This difference is stored in 
		// Theta_error, and added to estimated theta (smc1.Theta) so the 
		// effective angle used for commutating the motor is the same at 
		// the end of open loop, and at the begining of closed loop. 
		// This Theta_error is then substracted from estimated theta 
		// gradually in increments of 0.05 degrees until the error is less
		// than 0.05 degrees.  0.05/180=0.0002777
		ParkParm.qAngle = smc1.Theta + Theta_error;
		if (fabsf(Theta_error) > 0.0002777f)
		{
			if (Theta_error < 0)
				Theta_error += 0.00027777f;
			else
				Theta_error -= 0.00027777f;
		}


	}
	return;
}
void InvPark(void)
{
	ParkParm.qValpha =  ParkParm.qVd*cosf(ParkParm.qAngle) - ParkParm.qVq*sinf(ParkParm.qAngle);
	ParkParm.qVbeta  =  ParkParm.qVd*sinf(ParkParm.qAngle) + ParkParm.qVq*cosf(ParkParm.qAngle);
	return;
}
void CalcRefVec(void)
{
     //SVGenParm.qVr1 =ParkParm.qValpha;
     //SVGenParm.qVr2 = (-ParkParm.qValpha + SQRT3 * ParkParm.qVbeta)/2;
     //SVGenParm.qVr3 = (-ParkParm.qValpha  - SQRT3 * ParkParm.qVbeta)/2;

    SVGenParm.qVr1 =ParkParm.qVbeta;
    SVGenParm.qVr2 = (-ParkParm.qVbeta + SQRT3 * ParkParm.qValpha)/2;
    SVGenParm.qVr3 = (-ParkParm.qVbeta  - SQRT3 * ParkParm.qValpha)/2;

     return;
}
void CalcTimes(void)
{
 
	//SVGenParm.iPWMPeriod = LOOPINTCY;

	//SVGenParm.T1 = ((float)SVGenParm.iPWMPeriod * SVGenParm.T1);
	//SVGenParm.T2 = ((float)SVGenParm.iPWMPeriod * SVGenParm.T2);
	//SVGenParm.Tc = (((float)SVGenParm.iPWMPeriod-SVGenParm.T1-SVGenParm.T2)/2);
	//SVGenParm.Tb = SVGenParm.Tc + SVGenParm.T1;
	//SVGenParm.Ta = SVGenParm.Tb + SVGenParm.T2 ;

	SVGenParm.T1 = 0.85f * SVGenParm.T1;
	SVGenParm.T2 = 0.85f * SVGenParm.T2;
	SVGenParm.Tc = (0.85f - SVGenParm.T1 - SVGenParm.T2)/2;
	SVGenParm.Tb = SVGenParm.Tc + SVGenParm.T1;
	SVGenParm.Ta = SVGenParm.Tb + SVGenParm.T2 ;


}  


void update_timer_Duty(double duty_A,double duty_B,double duty_C)
{
	OUT[0] = duty_A;
	OUT[1] = duty_B;
	OUT[2] = duty_C;
}
void CalcSVGen( void )
{ 
	if( SVGenParm.qVr1 >= 0 )
	{       
		// (xx1)
		if( SVGenParm.qVr2 >= 0 )
		{
			// (x11)
			// Must be Sector 3 since Sector 7 not allowed
			// Sector 3: (0,1,1)  0-60 degrees
			SVGenParm.T2 = SVGenParm.qVr2;
			SVGenParm.T1 = SVGenParm.qVr1;
			CalcTimes();
			update_timer_Duty(SVGenParm.Ta,SVGenParm.Tb,SVGenParm.Tc) ;
			testpoint = 3;
		}
		else
		{            
			// (x01)
			if( SVGenParm.qVr3 >= 0 )
			{
				// Sector 5: (1,0,1)  120-180 degrees
				SVGenParm.T2 = SVGenParm.qVr1;
				SVGenParm.T1 = SVGenParm.qVr3;
				CalcTimes();
				update_timer_Duty(SVGenParm.Tc,SVGenParm.Ta,SVGenParm.Tb) ;
				testpoint = 5;

			}
			else
			{
				// Sector 1: (0,0,1)  60-120 degrees
				SVGenParm.T2 = -SVGenParm.qVr2;
				SVGenParm.T1 = -SVGenParm.qVr3;
				CalcTimes();
				update_timer_Duty(SVGenParm.Tb,SVGenParm.Ta,SVGenParm.Tc) ;
				testpoint = 1;
			}
		}
	}
	else
	{
		// (xx0)
		if( SVGenParm.qVr2 >= 0 )
		{
			// (x10)
			if( SVGenParm.qVr3 >= 0 )
			{
				// Sector 6: (1,1,0)  240-300 degrees
				SVGenParm.T2 = SVGenParm.qVr3;
				SVGenParm.T1 = SVGenParm.qVr2;
				CalcTimes();
				update_timer_Duty(SVGenParm.Tb,SVGenParm.Tc,SVGenParm.Ta) ;
				testpoint = 6;
			}
			else
			{
				// Sector 2: (0,1,0)  300-0 degrees
				SVGenParm.T2 = -SVGenParm.qVr3;
				SVGenParm.T1 = -SVGenParm.qVr1;
				CalcTimes();
				update_timer_Duty(SVGenParm.Ta,SVGenParm.Tc,SVGenParm.Tb) ;
				testpoint = 2;
			}
		}
		else
		{            
			// (x00)
			// Must be Sector 4 since Sector 0 not allowed
			// Sector 4: (1,0,0)  180-240 degrees
			SVGenParm.T2 = -SVGenParm.qVr1;
			SVGenParm.T1 = -SVGenParm.qVr2;
			CalcTimes();
			update_timer_Duty(SVGenParm.Tc,SVGenParm.Tb,SVGenParm.Ta) ;
			testpoint = 4;
		}
	}

}



__declspec(dllexport) void simuser (double t, double delt, double *in, double *out)
{
// Place your code here............begin
//  Define "sum" as "static" in order to retain its value.
    static double clk0=0.,clk1=0.;
	static int Initialize_flag=1;
	static float theta=0.0;


	if(Initialize_flag)
	{
		SMCInit(&smc1);
   		SetupControlParameters(); 
		FWInit();
		SetupParm();

	        uGF.bit.ChangeSpeed = 0;
	        uGF.bit.OpenLoop = 0;           // start in openloop
	        uGF.bit.RunMotor = 1;

		uGF.bit.EnTorqueMod = 0;
		uGF.bit.EnVoltRipCo =0;

		Initialize_flag = 0;

	PIParmW.qKp =  (float)in[4];		 
	PIParmW.qKi = (float)in[5];	

	PIParmD.qKp = (float)in[6];		 
	PIParmD.qKi = (float)in[7];	

	PIParmQ.qKp = (float)in[8];		 
	PIParmQ.qKi = (float)in[9];	

	
		
	}


	IN[0] = in[0];
	IN[1] = in[1];
	IN[2] = in[2];
	IN[3] = in[3];

	IN[10] = in[10];

	

	clk0 = in[11];
	if(!clk0 && !clk1) //LOW
	{

	}
	else if(clk0 && !clk1)// raising edge
	{
		
		if( uGF.bit.RunMotor )
		{
			// Calculate qIa,qIb
			//MeasCompCurr();

			//ParkParm.qIa = (float)IN[0];
			//ParkParm.qIb = (float)IN[1];
			//ParkParm.qAngle = (float)IN[2];
			//smc1.Omega = (float)IN[3];
			//smc1.Omega = (float)IN[3] *LOOPTIMEINSEC * IRP_PERCALC * POLEPAIRS/PI;
			// Calculate commutation angle using estimator
			//CalculateParkAngle();

			SpeedReference = (float)IN[10];

			
			//theta+= 0.02f;
			//if(  theta < (8*PI))ParkParm.qAngle=theta;
	
			// Calculate qId,qIq from qSin,qCos,qIa,qIb
			//ClarkePark();
					   
			// Calculate control values
			//DoControl();

			ParkParm.qVd =0.95f;
			ParkParm.qVq = 0.95f;

			//ParkParm.qAngle-= 0.002f;
			//if(  ParkParm.qAngle < 0)ParkParm.qAngle=2*PI;

			ParkParm.qAngle += 0.002f;
			if(2*PI <  ParkParm.qAngle)ParkParm.qAngle=0.0f;

			// Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
			InvPark();

			// Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta 
			CalcRefVec();

			// Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
			CalcSVGen();

	
		} 

	}
	else if(!clk0 && clk1)// falling edge
	{

	}
	else if(clk0 && clk1)//HIGH
	{

	}
	clk1 = clk0;


	out[0] = OUT[0];
	out[1] = OUT[1];
	out[2] = OUT[2];
	
	/*out[3] = ParkParm.qIa;
	out[4] = ParkParm.qIb;
	out[5] = ParkParm.qIalpha;
	out[6] = ParkParm.qIbeta;
	out[7] = ParkParm.qId;
	out[8] = ParkParm.qIq;
	out[9] = ParkParm.qVd;
	out[10] = ParkParm.qVq;
	out[11] = ParkParm.qAngle ;*/

	out[3] = PIParmW.qInMeas;//60
	out[4] = PIParmW.qInRef ;//61
	out[5] = PIParmW.qOut;//62
	out[6] = PIParmD.qInMeas;//63
	out[7] = PIParmD.qInRef;//64
	out[8] = ParkParm.qVd;//65
	out[9] = PIParmQ.qInMeas;//66
	out[10] =  PIParmQ.qInRef;//67
	out[11] = ParkParm.qVq;//68*/


	/*out[3] = SVGenParm.qVr1 ;//60
	out[4] = SVGenParm.qVr2;//61
	out[5] =SVGenParm.qVr3;//62
	//out[6] =  SVGenParm.T2_duty;//63
	out[7] = SVGenParm.Ta;//64
	out[8] = SVGenParm.Tb;//65
	out[9] = SVGenParm.Tc;//66
	out[10] =  testpoint;//67
	out[11] = testpoint;//68*/


// Place your code here............end
}
