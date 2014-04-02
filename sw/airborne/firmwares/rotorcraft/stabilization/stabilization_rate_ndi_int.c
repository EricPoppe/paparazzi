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

/* Debug variables REMOVE*/
float test;

#include "stabilization_rate_ndi_int.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"

#include "std.h"
#include <stdlib.h>

struct Int32Rates prev_body_rate; ///< with INT32_RATE_FRAC

#define RATE_GAINS_FRAC 4

struct Int32NDIRateGains rate_ndi_gains = {
	{STABILIZATION_RATE_NDI_P_PGAIN, STABILIZATION_RATE_NDI_Q_PGAIN, STABILIZATION_RATE_NDI_R_PGAIN},
	{STABILIZATION_RATE_NDI_P_IGAIN,STABILIZATION_RATE_NDI_Q_IGAIN, STABILIZATION_RATE_NDI_R_IGAIN},
	{STABILIZATION_RATE_NDI_P_DGAIN,STABILIZATION_RATE_NDI_Q_DGAIN, STABILIZATION_RATE_NDI_R_DGAIN},
	{STABILIZATION_RATE_NDI_P_DDGAIN,STABILIZATION_RATE_NDI_Q_DDGAIN, STABILIZATION_RATE_NDI_R_DDGAIN}
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
  (STABILIZATION_RATE_NDI_P_DDGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_Q_DDGAIN < 0)   ||   \
  (STABILIZATION_RATE_NDI_R_DDGAIN < 0)
#warning "ALL control gains are now positive!!!"
#endif

struct Int32Rates stabilization_rate_ndi_sum_err; ///< INT32_RATE_FRAC

struct Int32VirtualInput virtual_input_ff;
struct Int32VirtualInput virtual_input_fb;
struct Int32VirtualInput virtual_input;
struct Int32Thrust desired_thrust;

#define VIRTUAL_INPUT_FRAC 16
#define RATE_TIMESCALED_FRAC 20

#define GAIN_PRESCALER_FF 1
#define GAIN_PRESCALER_P 1
#define GAIN_PRESCALER_I 1
#define GAIN_PRESCALER_D 1
#define GAIN_PRESCALER_DD 1

void stabilization_rate_ndi_init(void) {

  stabilization_rate_ref_init();
  INT_RATES_ZERO(stabilization_rate_ndi_sum_err);
  INT_RATES_ZERO(prev_body_rate);

}

static void rate_ndi_run_ff(struct Int32VirtualInput *input, struct Int32NDIRateGains *gains, struct Int32Rates *ref_accel)
{
  /* Compute feedforward based on reference acceleration */

  input->p = (GAIN_PRESCALER_FF * gains->dd.p * (ref_accel->p)) >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC);
  input->q = (GAIN_PRESCALER_FF * gains->dd.q * (ref_accel->q)) >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC);
  input->r = (GAIN_PRESCALER_FF * gains->dd.r * (ref_accel->r)) >> (RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC);

}


static void rate_ndi_run_fb(struct Int32VirtualInput *input, struct Int32NDIRateGains *gains, struct Int32Rates *rate_ndi_err,
    struct Int32Rates *accel_err, struct Int32Rates *sum_err)
{
  /*  PID feedback */
  input->p =
    ((GAIN_PRESCALER_P * gains->p.p  * (rate_ndi_err->p)) >> (RATE_GAINS_FRAC + INT32_RATE_FRAC - VIRTUAL_INPUT_FRAC)) +
    ((GAIN_PRESCALER_D * gains->d.p  * (accel_err->p)) >> (RATE_GAINS_FRAC + RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC)) +
    ((GAIN_PRESCALER_I * gains->i.p  * (sum_err->p)) >> (RATE_GAINS_FRAC + RATE_TIMESCALED_FRAC - VIRTUAL_INPUT_FRAC));

  input->q =
    ((GAIN_PRESCALER_P * gains->p.q  * (rate_ndi_err->q)) >> (RATE_GAINS_FRAC + INT32_RATE_FRAC - VIRTUAL_INPUT_FRAC)) +
    ((GAIN_PRESCALER_D * gains->d.q  * (accel_err->q)) >> (RATE_GAINS_FRAC + RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC)) +
    ((GAIN_PRESCALER_I * gains->i.q  * (sum_err->q)) >> (RATE_GAINS_FRAC + RATE_TIMESCALED_FRAC - VIRTUAL_INPUT_FRAC));

  input->r =
    ((GAIN_PRESCALER_P * gains->p.r  * (rate_ndi_err->r)) >> (RATE_GAINS_FRAC + INT32_RATE_FRAC - VIRTUAL_INPUT_FRAC)) +
    ((GAIN_PRESCALER_D * gains->d.r  * (accel_err->r)) >> (RATE_GAINS_FRAC + RATE_REF_ACCEL_FRAC - VIRTUAL_INPUT_FRAC)) +
    ((GAIN_PRESCALER_I * gains->i.r  * (sum_err->r)) >> (RATE_GAINS_FRAC + RATE_TIMESCALED_FRAC - VIRTUAL_INPUT_FRAC));

}

static void rate_ndi_run_accel_to_thrust(struct Int32Thrust *thrust, struct Int32VirtualInput *input)
{

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
  stab_rate_ref_scaled.p = stab_rate_ref.p >> (RATE_REF_RATE_FRAC - INT32_RATE_FRAC);
  stab_rate_ref_scaled.q = stab_rate_ref.q >> (RATE_REF_RATE_FRAC - INT32_RATE_FRAC);
  stab_rate_ref_scaled.r = stab_rate_ref.r >> (RATE_REF_RATE_FRAC - INT32_RATE_FRAC);
  RATES_DIFF(rate_ndi_err, *body_rate, stab_rate_ref_scaled);

  /* angular acceleration */
  struct Int32Rates accel_err; ///< with RATE_REF_ACCEL_FRAC
  struct Int32Rates body_accel; ///< with RATE_REF_ACCEL_FRAC
  struct Int32Rates rate_diff; ///< with INT32_RATE_FRAC
  RATES_DIFF(rate_diff, *body_rate, prev_body_rate);
  RATES_COPY(prev_body_rate, *body_rate);
  body_accel.p = rate_diff.p << (F_UPDATE_RES - INT32_RATE_FRAC + RATE_REF_ACCEL_FRAC);
  body_accel.q = rate_diff.q << (F_UPDATE_RES - INT32_RATE_FRAC + RATE_REF_ACCEL_FRAC);
  body_accel.r = rate_diff.r << (F_UPDATE_RES - INT32_RATE_FRAC + RATE_REF_ACCEL_FRAC);

  /* error derivative */
  RATES_DIFF(accel_err, stab_rate_ref_accel, body_accel);
  struct Int32Rates timescaled_rate_err; ///< RATE_TIMESCALED_FRAC
  /* integrated error */
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

  /* compute the feed forward command */
  rate_ndi_run_ff(&virtual_input_ff, &rate_ndi_gains, &stab_rate_ref_accel);

  /* compute the feed back command */
  rate_ndi_run_fb(&virtual_input_fb, &rate_ndi_gains, &rate_ndi_err, &accel_err, &stabilization_rate_ndi_sum_err);

  /* sum feedforward and feedback */
  virtual_input.p = virtual_input_fb.p + virtual_input_ff.p;
  virtual_input.q = virtual_input_fb.q + virtual_input_ff.q;
  virtual_input.r = virtual_input_fb.r + virtual_input_ff.r;

  /* compute thrust from desired angular acceleration */
  rate_ndi_run_accel_to_thrust(&desired_thrust, &virtual_input);

  test = FLOAT_OF_BFP(virtual_input.q,VIRTUAL_INPUT_FRAC);

}
