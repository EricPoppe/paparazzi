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

struct Int32Rates prev_body_rate; ///< with INT32_RATE_FRAC

#define RATE_GAINS_FRAC 7

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

#define VIRTUAL_INPUT_FRAC 12
#define RATE_TIMESCALED_FRAC 21

#define GAIN_PRESCALER_FF 1
#define GAIN_PRESCALER_P 1
#define GAIN_PRESCALER_I 1
#define GAIN_PRESCALER_D 1
#define GAIN_PRESCALER_FF 1

void stabilization_rate_ndi_init(void) {

  stabilization_rate_ref_init();
  INT_RATES_ZERO(stabilization_rate_ndi_sum_err);
  INT_RATES_ZERO(prev_body_rate);

  rate_thrust_diff.roll = 0;
  rate_thrust_diff.pitch = 0;
  rate_thrust_diff.yaw = 0;

}

static void rate_ndi_run_ff(struct Int32VirtualInput *input, struct Int32NDIRateGains *gains, struct Int32Rates *ref_accel)
{
  /* Compute feedforward based on reference acceleration */

  input->p = (GAIN_PRESCALER_FF * gains->ff.p * (ref_accel->p));
  input->q = (GAIN_PRESCALER_FF * gains->ff.q * (ref_accel->q));
  input->r = (GAIN_PRESCALER_FF * gains->ff.r * (ref_accel->r));

}


static void rate_ndi_run_fb(struct Int32VirtualInput *input, struct Int32NDIRateGains *gains, struct Int32Rates *rate_ndi_err,
    struct Int32Rates *accel_err, struct Int32Rates *sum_err)
{
  /*  PID feedback */
  input->p =
    GAIN_PRESCALER_P * INT_MULT_RSHIFT(gains->p.p, rate_ndi_err->p, RATE_GAINS_FRAC) +
    GAIN_PRESCALER_D * INT_MULT_RSHIFT(gains->d.p, accel_err->p, RATE_GAINS_FRAC) +
    GAIN_PRESCALER_I * INT_MULT_RSHIFT(gains->i.p , sum_err->p, RATE_GAINS_FRAC);

  input->q =
      GAIN_PRESCALER_P * INT_MULT_RSHIFT(gains->p.q, rate_ndi_err->q, RATE_GAINS_FRAC) +
      GAIN_PRESCALER_D * INT_MULT_RSHIFT(gains->d.q, accel_err->q, RATE_GAINS_FRAC) +
      GAIN_PRESCALER_I * INT_MULT_RSHIFT(gains->i.q , sum_err->q, RATE_GAINS_FRAC);

  input->r =
      GAIN_PRESCALER_P * INT_MULT_RSHIFT(gains->p.r, rate_ndi_err->r, RATE_GAINS_FRAC) +
      GAIN_PRESCALER_D * INT_MULT_RSHIFT(gains->d.r, accel_err->r, RATE_GAINS_FRAC) +
      GAIN_PRESCALER_I * INT_MULT_RSHIFT(gains->i.r , sum_err->r, RATE_GAINS_FRAC);

}

static void rate_ndi_run_accel_to_thrust(struct Int32ThrustDiff *thrust_diff, struct Int32VirtualInput *input)
{
	int32_t Mx; // desired moment around x-axis in INT32_STAB_ALT_T_FRAC
	int32_t My; // desired moment around x-axis in INT32_STAB_ALT_T_FRAC
	int32_t Mz; // desired moment around x-axis in INT32_STAB_ALT_T_FRAC

	Mx = (int32_t)((input->p << (INT32_STAB_ALT_T_FRAC - VIRTUAL_INPUT_FRAC))*STABILIZATION_NDI_IXX);
	My = (int32_t)((input->q << (INT32_STAB_ALT_T_FRAC - VIRTUAL_INPUT_FRAC))*STABILIZATION_NDI_IXX);
	Mz = (int32_t)((input->r << (INT32_STAB_ALT_T_FRAC - VIRTUAL_INPUT_FRAC))*STABILIZATION_NDI_IXX);

	thrust_diff->roll = Mx/STABILIZATION_NDI_ARM;
	thrust_diff->pitch = My/STABILIZATION_NDI_ARM;
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

  struct Int32Rates rate_err_scaled; // with VIRTUAL_INPUT_FRAC
  rate_err_scaled.p = (rate_ndi_err.p >> (INT32_RATE_FRAC - VIRTUAL_INPUT_FRAC));
  rate_err_scaled.q = (rate_ndi_err.q >> (INT32_RATE_FRAC - VIRTUAL_INPUT_FRAC));
  rate_err_scaled.r = (rate_ndi_err.r >> (INT32_RATE_FRAC - VIRTUAL_INPUT_FRAC));

  /* angular acceleration */
  struct Int64Rates accel_err; ///< with RATE_REF_ACCEL_FRAC
  struct Int64Rates body_accel; ///< with RATE_REF_ACCEL_FRAC
  struct Int32Rates rate_diff; ///< with INT32_RATE_FRAC
  RATES_DIFF(rate_diff, *body_rate, prev_body_rate);
  RATES_COPY(prev_body_rate, *body_rate);
  body_accel.p = ((int64_t)rate_diff.p) << (F_UPDATE_RES - INT32_RATE_FRAC + RATE_REF_ACCEL_FRAC);
  body_accel.q = ((int64_t)rate_diff.q) << (F_UPDATE_RES - INT32_RATE_FRAC + RATE_REF_ACCEL_FRAC);
  body_accel.r = ((int64_t)rate_diff.r) << (F_UPDATE_RES - INT32_RATE_FRAC + RATE_REF_ACCEL_FRAC);

  /* error derivative */
  struct Int32Rates accel_err_scaled; // with VIRTUAL_INPUT_FRAC
  RATES_DIFF(accel_err, stab_rate_ref_accel, body_accel);
  accel_err_scaled.p = (int32_t)(accel_err.p >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC));
  accel_err_scaled.q = (int32_t)(accel_err.q >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC));
  accel_err_scaled.r = (int32_t)(accel_err.r >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC));

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

  struct Int32Rates sum_err_scaled; // with VIRTUAL_INPUT_FRAC
  sum_err_scaled.p = (stabilization_rate_ndi_sum_err.p >> (RATE_TIMESCALED_FRAC - VIRTUAL_INPUT_FRAC));
  sum_err_scaled.q = (stabilization_rate_ndi_sum_err.q >> (RATE_TIMESCALED_FRAC - VIRTUAL_INPUT_FRAC));
  sum_err_scaled.r = (stabilization_rate_ndi_sum_err.r >> (RATE_TIMESCALED_FRAC - VIRTUAL_INPUT_FRAC));

  /* compute the feed forward command */
  struct Int32Rates stab_rate_ref_accel_scaled; // with VIRTUAL_INPUT_FRAC
  stab_rate_ref_accel_scaled.p = (int32_t)(stab_rate_ref_accel.p >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC));
  stab_rate_ref_accel_scaled.q = (int32_t)(stab_rate_ref_accel.q >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC));
  stab_rate_ref_accel_scaled.r = (int32_t)(stab_rate_ref_accel.r >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC));
  rate_ndi_run_ff(&virtual_input_ff, &rate_ndi_gains, &stab_rate_ref_accel_scaled);

  /* compute the feed back command */
  rate_ndi_run_fb(&virtual_input_fb, &rate_ndi_gains, &rate_err_scaled, &accel_err_scaled, &sum_err_scaled);

  /* sum feedforward and feedback */
  virtual_input.p = virtual_input_fb.p + virtual_input_ff.p;
  virtual_input.q = virtual_input_fb.q + virtual_input_ff.q;
  virtual_input.r = virtual_input_fb.r + virtual_input_ff.r;

  /* compute thrust from desired angular acceleration */
  rate_ndi_run_accel_to_thrust(&rate_thrust_diff, &virtual_input);

}
