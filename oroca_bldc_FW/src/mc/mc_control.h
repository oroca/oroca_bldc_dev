/*
	Copyright 2012-2014 OROCA ESC Project 	www.oroca.org

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * mc_pwm.h
 *
 *  Created on: 13 okt 2012
 *      Author: bakchajang
 */

#ifndef _MC_CONTROL_H_
#define _MC_CONTROL_H_

//======================================================================================
//macro
#define Digital_PI_controller(out, ref, in, err0, limit, kp, ki, tsample)   \
		{						                                            \
			float err, tmp_kp, tmp_kpi;                                     \
			tmp_kp = (float)(kp);                                           \
			tmp_kpi = (float)(kp + ki*tsample);                             \
			err = ref - in;					                                \
			out += ((tmp_kpi * err) - (tmp_kp * err0));	                    \
			out = Bound_limit(out, limit);                                  \
			err0 = err;                                                     \
		}
#define Bound_limit(in,lim)	((in > (lim)) ? (lim) : ((in < -(lim)) ? -(lim) : in))
#define Bound_min_max(in, min, max)	((in > (max)) ? (max) : ((in < (min)) ? (min) : in))

#define Low_pass_filter(out, in, in_old, alpha)     	\
		{												\
			float tmp;									\
			tmp = alpha*(in + in_old - (out*2)); 		\
			out += tmp; 								\
			in_old = in;								\
		}

#define CalcPIctrlr(qOut,qInRef,qInMeas,qdSum,qKp,qKi,qOutMax,qKc,qOutMin)	\
		{																	\
			float U,Exc,Err;												\
			Err  = qInRef - qInMeas;										\
			U  = qdSum + qKp * Err;											\
			qOut = Bound_min_max(U, qOutMin, qOutMax);						\
			Exc = U - qOut;													\
			qdSum = qdSum + qKi * Err - qKc * Exc ;							\
		}

//==========================================================================
//mc internal use only
extern tPIParm PIParmD;
extern tPIParm PIParmQ;
extern tPIParm PIParmW;	
extern tPIParm PIParmPLL;

extern tMcCtrlBits McCtrlBits;
extern tCtrlParm CtrlParm;
extern tParkParm ParkParm;
extern tSVGenParm SVGenParm;
extern tFdWeakParm FdWeakParm;

//public function Declaration
#ifdef __cplusplus
extern "C" {
#endif

	void SpeedControl( void );
	void CurrentControl( void );



#ifdef __cplusplus
}
#endif


#endif
