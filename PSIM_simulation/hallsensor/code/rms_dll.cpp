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
#define Bound_limit(in,lim)	((in > (lim)) ? (lim) : ((in < -(lim)) ? -(lim) : in))

#define Fsamp           16000
#define Tsamp           1./16000

#define		PI				3.14159265358979f

float HallPLLlead      = 0.0;
float HallPLLlead1     = 0.0;
float HallPLLlead2     = 0.0;
float HallPLLqe        = 0.0;
float HallPLLde        = 0.0;
float HallPLLde1       = 0.0;
float HallPLLdef       = 0.0;
float HallPLLdef1      = 0.0;
#define WMd      2.*3.141592654*180.
#define AMd      (WMd-(2./Tsamp))/(WMd+(2./Tsamp))
#define BMd      WMd/(WMd+(2./Tsamp))
	
static volatile float Theta	 	= 0.0;
static volatile float ThetaCal	 	= 0.0;

static volatile float Futi	 	= 0.0;
float Wpll	 	= 0.0;
float Wpll1	 	= 0.0;
float Wpllp	 	= 0.0;
float Wplli	 	= 0.0;

float Kpll       = 0.428;
float Ipll       = 28.83;


static  float Hall_KA = 0.0;
static  float Hall_KB = 0.0;

static  float Hall_PIout = 0.0;
static  float Hall_Err0 = 0.0;

float HallPLLA	 = 0.0f;	
float HallPLLA1 	= 0.0f;
float HallPLLB	   = 0.0f;

float HallPLLA_cos3th	 = 0.0f;
float HallPLLA_sin3th	 = 0.0f;
float HallPLLB_sin3th	   = 0.0f;
float HallPLLB_cos3th	   = 0.0f;

float HallPLLA_cos3th_Integral = 0.0f;
float HallPLLA_sin3th_Integral = 0.0f;
float HallPLLB_sin3th_Integral  = 0.0f;
float HallPLLB_cos3th_Integral = 0.0f;

float HallPLLA_old = 0.0f;
float HallPLLB_old = 0.0f;

float HallPLLA_filtered = 0.0f;
float HallPLLB_filtered = 0.0f;

float Hall_SinCos;
float Hall_CosSin;

float Gamma = 1.0f;

float costh;
float sinth;

float Asin3th = 0.0f;
float Acos3th = 0.0f;
float Bsin3th= 0.0f;
float Bcos3th= 0.0f;
float ANF_PLLA= 0.0f;
float ANF_PLLB= 0.0f;

float cos3th;
float sin3th;

float wt=0.0f;
float sinwt=0.0f;

static double IN[12],OUT[12];

__declspec(dllexport) void simuser (double t, double delt, double *in, double *out)
{
// Place your code here............begin
//  Define "sum" as "static" in order to retain its value.
    static double clk0=0.,clk1=0.;

	
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
		HallPLLA = in[0];
		HallPLLB = in[1];

		cos3th = cosf(3.0f * Theta);
		sin3th = sinf(3.0f * Theta);

		HallPLLA_sin3th = HallPLLA * sin3th * Gamma;
		HallPLLA_cos3th = HallPLLA * cos3th * Gamma;
	
		HallPLLB_cos3th = HallPLLB* cos3th * Gamma;
		HallPLLB_sin3th = HallPLLB * sin3th * Gamma;

		HallPLLA_cos3th_Integral += HallPLLA_cos3th;
		HallPLLA_sin3th_Integral += HallPLLA_sin3th;
	
		HallPLLB_sin3th_Integral += HallPLLB_sin3th;
		HallPLLB_cos3th_Integral += HallPLLB_cos3th;

		Asin3th= HallPLLA_sin3th_Integral * sin3th;
		Acos3th= HallPLLA_cos3th_Integral * cos3th;

		Bsin3th= HallPLLB_sin3th_Integral * sin3th;
		Bcos3th= HallPLLB_cos3th_Integral * cos3th;

		ANF_PLLA = HallPLLA - Asin3th - Acos3th;
		ANF_PLLB = HallPLLB - Bsin3th - Bcos3th;
	
		costh = cosf(Theta);
		sinth = sinf(Theta);
	
		//Hall_SinCos = ANF_PLLA * costh;
		//Hall_CosSin = ANF_PLLB * sinth;

		Hall_SinCos = HallPLLA * costh;
		Hall_CosSin = HallPLLB * sinth;

		float err, tmp_kp, tmp_kpi; 									
		tmp_kp = 0.5f;
		tmp_kpi = (1.0f + 2.01f * Tsamp);
		err = Hall_SinCos - Hall_CosSin; 											
		Hall_PIout += ((tmp_kpi * err) - (tmp_kp * Hall_Err0)); 					
		Hall_PIout = Bound_limit(Hall_PIout, 10.0f);						
		Hall_Err0= err;									
	
		Theta += Hall_PIout ;
		if((2.0f * PI) < Theta) Theta = Theta - (2.0f * PI);
		else if(Theta < 0.0f) Theta = (2.0f * PI) + Theta;

		sinwt = sinf(wt);
		wt = wt + 0.01f;
		if(2*PI < wt)wt=0;
	}
	else if(!clk0 && clk1)// falling edge
	{

	}
	else if(clk0 && clk1)//HIGH
	{

	}
	clk1 = clk0;


	out[0] = Theta;
	out[1] = costh;
	out[2] = sinth;
	
	out[3] =Hall_SinCos;
	out[4] = Hall_CosSin;
	out[5] = HallPLLA;
	out[6] = HallPLLB;
	out[7] = sinwt;
	/*out[8] = ParkParm.qIq;
	out[9] = ParkParm.qVd;
	out[10] = ParkParm.qVq;
	out[11] = ParkParm.qAngle ;*/


// Place your code here............end
}
