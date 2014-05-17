/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_rate_ndi_quat_int.c
 * Rotorcraft quaternion rate stabilization with NDI control
 */

#include "stabilization_rate_ndi_int.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"

#include "std.h"
#include <stdlib.h>

/*REMOVE DEBUG*/
float rate_test;
float test = 0;

struct Int32Rates prev_body_rate; ///< with INT32_RATE_FRAC
struct Int64Rates body_accel; ///< with RATE_REF_ACCEL_FRAC

#define RATE_GAINS_FRAC 7
#define INT32_STAB_RATE_M_FRAC 20

/* body parameters TODO: FIND REAL VALUES*/
#ifndef STABILIZATION_NDI_IXX
#define STABILIZATION_NDI_IXX 0.00225
#endif

#ifndef STABILIZATION_NDI_IYY
#define STABILIZATION_NDI_IYY 0.00225
#endif

#ifndef STABILIZATION_NDI_IZZ
#define STABILIZATION_NDI_IZZ 0.0045
#endif

#ifndef STABILIZATION_NDI_ARM
#define STABILIZATION_NDI_ARM 0.15
#endif

struct Int32NDIRateGains rate_ndi_gains = {
	{STABILIZATION_RATE_NDI_P_PGAIN, STABILIZATION_RATE_NDI_Q_PGAIN, STABILIZATION_RATE_NDI_R_PGAIN},
	{STABILIZATION_RATE_NDI_P_IGAIN,STABILIZATION_RATE_NDI_Q_IGAIN, STABILIZATION_RATE_NDI_R_IGAIN},
	{STABILIZATION_RATE_NDI_P_DGAIN,STABILIZATION_RATE_NDI_Q_DGAIN, STABILIZATION_RATE_NDI_R_DGAIN},
	{STABILIZATION_RATE_NDI_P_FFGAIN,STABILIZATION_RATE_NDI_Q_FFGAIN, STABILIZATION_RATE_NDI_R_FFGAIN}
};

/* warn if some gains are still negative */
#if (STABILIZATION_RATE_NDI_P_PGAIN < 0) ||   \
  (STABILIZATION_RATE_NDI_Q_PGAIN < 0) ||   \
  (STABILIZATION_RATE_NDI_R_PGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_P_IGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_Q_IGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_R_IGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_P_DGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_Q_DGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_R_DGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_P_FFGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_Q_FFGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_R_FFGAIN < 0)
#warning "ALL control gains are now positive!!!"
#endif

struct Int32Rates stabilization_rate_ndi_sum_err; ///< RATE_TIMESCALED_FRAC

struct Int32VirtualInput virtual_input_ff; // VIRTUAL_INPUT_FRAC
struct Int32VirtualInput virtual_input_fb; // VIRTUAL_INPUT_FRAC
struct Int32VirtualInput virtual_input; // VIRTUAL_INPUT_FRAC
struct Int32Thrust rate_thrust; // INT32_STAB_ALT_T_FRAC

#define RATE_TIMESCALED_FRAC 21

#define GAIN_PRESCALER_FF 1
#define GAIN_PRESCALER_P 1
#define GAIN_PRESCALER_I 1
#define GAIN_PRESCALER_D 0.1
#define GAIN_PRESCALER_FF 1

void stabilization_rate_ndi_init(void) {

  stabilization_rate_ref_init();
  INT_RATES_ZERO(stabilization_rate_ndi_sum_err);
  INT_RATES_ZERO(prev_body_rate);

  rate_thrust_diff.roll = 0;
  rate_thrust_diff.pitch = 0;
  rate_thrust_diff.yaw = 0;

	body_accel.p = 0;
	body_accel.q = 0;
	body_accel.r = 0;

}

static void rate_ndi_run_ff(struct Int32VirtualInput *input, struct Int32NDIRateGains *gains, struct Int32Rates *ref_accel)
{
  /* Compute feedforward based on reference acceleration */

  input->p = (GAIN_PRESCALER_FF * gains->ff.p * (ref_accel->p));
  input->q = (GAIN_PRESCALER_FF * gains->ff.q * (ref_accel->q));
  input->r = (GAIN_PRESCALER_FF * gains->ff.r * (ref_accel->r));

}


static void rate_ndi_run_fb(struct Int32VirtualInput *input, struct Int32NDIRateGains *gains, struct Int32Rates *rate_ndi_err,
    struct Int64Rates *accel_err, struct Int32Rates *sum_err)
{
  /*  PID feedback */
  input->p =
    GAIN_PRESCALER_P * INT_MULT_RSHIFT(gains->p.p, rate_ndi_err->p, RATE_GAINS_FRAC + INT32_RATE_FRAC - VIRTUAL_INPUT_FRAC) +
    (int32_t)(GAIN_PRESCALER_D * INT_MULT_RSHIFT(gains->d.p, accel_err->p, RATE_GAINS_FRAC + RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC)) +
    GAIN_PRESCALER_I * (int32_t)INT_MULT_RSHIFT((int64_t)gains->i.p , (int64_t)sum_err->p, RATE_GAINS_FRAC + RATE_TIMESCALED_FRAC - VIRTUAL_INPUT_FRAC);

  input->q =
    GAIN_PRESCALER_P * INT_MULT_RSHIFT(gains->p.q, rate_ndi_err->q, RATE_GAINS_FRAC + INT32_RATE_FRAC - VIRTUAL_INPUT_FRAC) +
    (int32_t)(GAIN_PRESCALER_D * INT_MULT_RSHIFT(gains->d.q, accel_err->q, RATE_GAINS_FRAC + RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC)) +
    GAIN_PRESCALER_I * (int32_t)INT_MULT_RSHIFT((int64_t)gains->i.q , (int64_t)sum_err->q, RATE_GAINS_FRAC + RATE_TIMESCALED_FRAC - VIRTUAL_INPUT_FRAC);

  input->r =
    GAIN_PRESCALER_P * INT_MULT_RSHIFT(gains->p.r, rate_ndi_err->r, RATE_GAINS_FRAC + INT32_RATE_FRAC - VIRTUAL_INPUT_FRAC) +
    (int32_t)(GAIN_PRESCALER_D * INT_MULT_RSHIFT(gains->d.r, accel_err->r, RATE_GAINS_FRAC + RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC)) +
    GAIN_PRESCALER_I * (int32_t)INT_MULT_RSHIFT((int64_t)gains->i.r , (int64_t)sum_err->r, RATE_GAINS_FRAC + RATE_TIMESCALED_FRAC - VIRTUAL_INPUT_FRAC);

}

static void rate_ndi_run_accel_to_thrust(struct Int32ThrustDiff *thrust_diff, struct Int32VirtualInput *input)
{
	int32_t Mx; // desired moment around x-axis in INT32_STAB_RATE_M_FRAC
	int32_t My; // desired moment around x-axis in INT32_STAB_RATE_M_FRAC
	int32_t Mz; // desired moment around x-axis in INT32_STAB_ALT_T_FRAC

	/*
	 * TODO: INT
	 */
	struct FloatMat33 I_mat;
	FLOAT_MAT33_DIAG(I_mat, STABILIZATION_NDI_IXX, STABILIZATION_NDI_IYY, STABILIZATION_NDI_IZZ);

	struct FloatVect3 omega_body;
	omega_body.x = stateGetBodyRates_f()->p;
	omega_body.y = stateGetBodyRates_f()->q;
	omega_body.z = stateGetBodyRates_f()->r;

	struct FloatVect3 Iomega;
	MAT33_VECT3_MUL(Iomega,I_mat,omega_body);

	struct FloatVect3 omegaIomega;
  VECT3_CROSS_PRODUCT(omegaIomega,omega_body,Iomega);

	struct FloatVect3 omega_d_body;
	omega_d_body.x = FLOAT_OF_BFP(input->p,VIRTUAL_INPUT_FRAC);
	omega_d_body.y = FLOAT_OF_BFP(input->q,VIRTUAL_INPUT_FRAC);
	omega_d_body.z = FLOAT_OF_BFP(input->r,VIRTUAL_INPUT_FRAC);

	struct FloatVect3 Iomega_d;
	MAT33_VECT3_MUL(Iomega_d,I_mat,omega_d_body);

	struct FloatVect3 Iomega_d_omegaIomega;

	Iomega_d_omegaIomega.x = Iomega_d.x + omegaIomega.x;
	Iomega_d_omegaIomega.y = Iomega_d.y + omegaIomega.y;
	Iomega_d_omegaIomega.z = Iomega_d.z + omegaIomega.z;

	Mx = BFP_OF_REAL(Iomega_d_omegaIomega.x,INT32_STAB_RATE_M_FRAC);
	My = BFP_OF_REAL(Iomega_d_omegaIomega.y,INT32_STAB_RATE_M_FRAC);
	Mz = BFP_OF_REAL(Iomega_d_omegaIomega.z,INT32_STAB_ALT_T_FRAC);

	thrust_diff->roll = BFP_OF_REAL(FLOAT_OF_BFP(Mx,INT32_STAB_RATE_M_FRAC)/STABILIZATION_NDI_ARM, INT32_STAB_ALT_T_FRAC);
	thrust_diff->pitch = BFP_OF_REAL(FLOAT_OF_BFP(My,INT32_STAB_RATE_M_FRAC)/STABILIZATION_NDI_ARM, INT32_STAB_ALT_T_FRAC);
	attitude_tdiff_from_tau_command(&thrust_diff->yaw, &Mz);

}

// CAUTION! Periodic frequency is assumed to be 512 Hz
// which is equal to >> 9
#define F_UPDATE_RES 9

void stabilization_rate_ndi_run(bool_t enable_integrator) {

  /*
   * Update reference
   */
  stabilization_rate_ref_update();

  /*
   * Compute errors for feedback
   */

  /* error                          */
  struct Int32Rates rate_ndi_err; ///< with INT32_RATE_FRAC
  struct Int32Rates* body_rate = stateGetBodyRates_i(); ///< with INT32_RATE_FRAC
  struct Int32Rates stab_rate_ref_scaled; ///< with INT32_RATE_FRAC
  stab_rate_ref_scaled.p = (int32_t)(stab_rate_ref.p >> (RATE_REF_RATE_FRAC - INT32_RATE_FRAC));
  stab_rate_ref_scaled.q = (int32_t)(stab_rate_ref.q >> (RATE_REF_RATE_FRAC - INT32_RATE_FRAC));
  stab_rate_ref_scaled.r = (int32_t)(stab_rate_ref.r >> (RATE_REF_RATE_FRAC - INT32_RATE_FRAC));
  RATES_DIFF(rate_ndi_err, stab_rate_ref_scaled, *body_rate);

  /* angular acceleration */
  struct Int64Rates accel_err; ///< with RATE_REF_ACCEL_FRAC
  struct Int32Rates rate_diff; ///< with INT32_RATE_FRAC

  if (body_rate->p != prev_body_rate.p) {
  	/*IMU runs at 200 hz*/
		RATES_DIFF(rate_diff, *body_rate, prev_body_rate);
		RATES_COPY(prev_body_rate, *body_rate);
		body_accel.p = (((int64_t)rate_diff.p) << (RATE_REF_ACCEL_FRAC - INT32_RATE_FRAC))*200;
		body_accel.q = (((int64_t)rate_diff.q) << (RATE_REF_ACCEL_FRAC - INT32_RATE_FRAC))*200;
		body_accel.r = (((int64_t)rate_diff.r) << (RATE_REF_ACCEL_FRAC - INT32_RATE_FRAC))*200;
  }

  /* error derivative */
  RATES_DIFF(accel_err, stab_rate_ref_accel, body_accel);

  /* integrated error */
  struct Int32Rates timescaled_rate_err; ///< RATE_TIMESCALED_FRAC
  if (enable_integrator) {
    /* update accumulator */
    timescaled_rate_err.p = rate_ndi_err.p >> (F_UPDATE_RES + INT32_RATE_FRAC - RATE_TIMESCALED_FRAC);
    timescaled_rate_err.q = rate_ndi_err.q >> (F_UPDATE_RES + INT32_RATE_FRAC - RATE_TIMESCALED_FRAC);
    timescaled_rate_err.r = rate_ndi_err.r >> (F_UPDATE_RES + INT32_RATE_FRAC - RATE_TIMESCALED_FRAC);
    RATES_ADD(stabilization_rate_ndi_sum_err, timescaled_rate_err);
  } else {
    /* reset accumulator */
    stabilization_rate_ndi_sum_err.p = 0;
    stabilization_rate_ndi_sum_err.q = 0;
    stabilization_rate_ndi_sum_err.r = 0;
  }

	/*limit integrator terms (just in case, shouldn't be neccesary due to PCH)*/
	if (stabilization_rate_ndi_sum_err.p > (1 << (29)))
		stabilization_rate_ndi_sum_err.p = (1 << (29));
	else if (stabilization_rate_ndi_sum_err.p < -(1 << (29)))
		stabilization_rate_ndi_sum_err.p = -(1 << (29));

	if (stabilization_rate_ndi_sum_err.q > (1 << (29)))
		stabilization_rate_ndi_sum_err.q = (1 << (29));
	else if (stabilization_rate_ndi_sum_err.q < -(1 << (29)))
		stabilization_rate_ndi_sum_err.q = -(1 << (29));

	if (stabilization_rate_ndi_sum_err.r > (1 << (29)))
		stabilization_rate_ndi_sum_err.r = (1 << (29));
	else if (stabilization_rate_ndi_sum_err.r < -(1 << (29)))
		stabilization_rate_ndi_sum_err.r = -(1 << (29));

  /* compute the feed forward command */
  struct Int32Rates stab_rate_ref_accel_scaled; // with VIRTUAL_INPUT_FRAC
  stab_rate_ref_accel_scaled.p = (int32_t)(stab_rate_ref_accel.p >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC));
  stab_rate_ref_accel_scaled.q = (int32_t)(stab_rate_ref_accel.q >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC));
  stab_rate_ref_accel_scaled.r = (int32_t)(stab_rate_ref_accel.r >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC));
  rate_ndi_run_ff(&virtual_input_ff, &rate_ndi_gains, &stab_rate_ref_accel_scaled);

  /* compute the feed back command */
  rate_ndi_run_fb(&virtual_input_fb, &rate_ndi_gains, &rate_ndi_err, &accel_err, &stabilization_rate_ndi_sum_err);

  /* sum feedforward and feedback */
  virtual_input.p = virtual_input_fb.p + virtual_input_ff.p;
  virtual_input.q = virtual_input_fb.q + virtual_input_ff.q;
  virtual_input.r = virtual_input_fb.r + virtual_input_ff.r;


//  /*DEBUG REMOVE*/
//  rate_test = FLOAT_OF_BFP(virtual_input_ff.p,VIRTUAL_INPUT_FRAC);

  /* compute thrust from desired angular acceleration */
  rate_ndi_run_accel_to_thrust(&rate_thrust_diff, &virtual_input);

}
