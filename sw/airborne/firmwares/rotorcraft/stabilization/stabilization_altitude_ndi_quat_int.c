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

/** @file stabilization_altitude_ndi_quat_int.c
 * Rotorcraft altitude stabilization with NDI control, using quaternions for tilt angle estimation
 */

#include "stabilization_altitude_ndi_quat_int.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "std.h"
#include <stdlib.h>
#include "mcu_periph/sys_time.h"

/*DEBUG REMOVE*/
float alt_test1;
float alt_test2;
float alt_test3;
float alt_test4;
float alt_test5;
//float z_sp;
//float z_d_sp;
//bool_t alt_sp;
//bool_t alt_d_sp;
int32_t time_counter_tmp = 0;

/* Controller gains */
struct Int32NDIAltitudeGains small_inner_gains = {
		STABILIZATION_ALTITUDE_NDI_SMALL_INNER_PGAIN, STABILIZATION_ALTITUDE_NDI_SMALL_INNER_IGAIN, STABILIZATION_ALTITUDE_NDI_SMALL_INNER_DGAIN, STABILIZATION_ALTITUDE_NDI_SMALL_INNER_FFGAIN
};

struct Int32NDIAltitudeGains small_outer_gains = {
		STABILIZATION_ALTITUDE_NDI_SMALL_OUTER_PGAIN, STABILIZATION_ALTITUDE_NDI_SMALL_OUTER_IGAIN, STABILIZATION_ALTITUDE_NDI_SMALL_OUTER_DGAIN, STABILIZATION_ALTITUDE_NDI_SMALL_OUTER_FFGAIN
};

struct Int32NDIAltitudeGains large_inner_gains = {
		STABILIZATION_ALTITUDE_NDI_LARGE_INNER_PGAIN, STABILIZATION_ALTITUDE_NDI_LARGE_INNER_IGAIN, STABILIZATION_ALTITUDE_NDI_LARGE_INNER_DGAIN, STABILIZATION_ALTITUDE_NDI_LARGE_INNER_FFGAIN
};

struct Int32NDIAltitudeGains large_outer_gains = {
		STABILIZATION_ALTITUDE_NDI_LARGE_OUTER_PGAIN, STABILIZATION_ALTITUDE_NDI_LARGE_OUTER_IGAIN, STABILIZATION_ALTITUDE_NDI_LARGE_OUTER_DGAIN, STABILIZATION_ALTITUDE_NDI_LARGE_OUTER_FFGAIN
};

/* warn if some gains are still negative */
#if (STABILIZATION_ALTITUDE_NDI_SMALL_INNER_PGAIN < 0) ||   \
		(STABILIZATION_ALTITUDE_NDI_SMALL_INNER_IGAIN < 0) ||   \
		(STABILIZATION_ALTITUDE_NDI_SMALL_INNER_DGAIN < 0)   ||   \
		(STABILIZATION_ALTITUDE_NDI_SMALL_INNER_FFGAIN < 0) || \
		(STABILIZATION_ALTITUDE_NDI_SMALL_OUTER_PGAIN < 0) ||   \
		(STABILIZATION_ALTITUDE_NDI_SMALL_OUTER_IGAIN < 0) ||   \
		(STABILIZATION_ALTITUDE_NDI_SMALL_OUTER_DGAIN < 0)   ||   \
		(STABILIZATION_ALTITUDE_NDI_SMALL_OUTER_FFGAIN < 0) ||  \
		(STABILIZATION_ALTITUDE_NDI_LARGE_INNER_PGAIN < 0) ||   \
		(STABILIZATION_ALTITUDE_NDI_LARGE_INNER_IGAIN < 0) ||   \
		(STABILIZATION_ALTITUDE_NDI_LARGE_INNER_DGAIN < 0)   ||   \
		(STABILIZATION_ALTITUDE_NDI_LARGE_INNER_FFGAIN < 0) || \
		(STABILIZATION_ALTITUDE_NDI_LARGE_OUTER_PGAIN < 0) ||   \
		(STABILIZATION_ALTITUDE_NDI_LARGE_OUTER_IGAIN < 0) ||   \
		(STABILIZATION_ALTITUDE_NDI_LARGE_OUTER_DGAIN < 0)   ||   \
		(STABILIZATION_ALTITUDE_NDI_LARGE_OUTER_FFGAIN < 0)
#warning "ALL control gains are now positive!!!"
#endif

#define STAB_ALT_ZD_ZETA_OMEGA_FRAC 10
#define STAB_ALT_ZD_OMEGA_2_FRAC 7
#define STAB_ALT_Z_ZETA_OMEGA_FRAC 10
#define STAB_ALT_Z_OMEGA_2_FRAC 7

/* second order reference model natural frequency and damping (small angle inner reference model)*/
//#ifndef STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_OMEGA
//#define STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_OMEGA RadOfDeg(100.)
//#endif
//#ifndef STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_ZETA
//#define STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_ZETA  0.85
//#endif
//#define STABILIZATION_ALTITUDE_NDI_SMALL_ZD_ZETA_OMEGA BFP_OF_REAL((STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_ZETA*STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_OMEGA), STAB_ALT_ZD_ZETA_OMEGA_FRAC)
//#define STABILIZATION_ALTITUDE_NDI_SMALL_ZD_OMEGA_2    BFP_OF_REAL((STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_OMEGA*STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_OMEGA), STAB_ALT_ZD_OMEGA_2_FRAC)

struct FloatAltRefModel small_inner_ref_model = {
		STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_OMEGA, STABILIZATION_ALTITUDE_NDI_SMALL_ZD_REF_ZETA
};

/* second order reference model natural frequency and damping (small angle outer reference model)*/
//#ifndef STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_OMEGA
//#define STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_OMEGA RadOfDeg(100.)
//#endif
//#ifndef STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_ZETA
//#define STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_ZETA  0.85
//#endif
//#define STABILIZATION_ALTITUDE_NDI_SMALL_Z_ZETA_OMEGA BFP_OF_REAL((STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_ZETA*STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_OMEGA), STAB_ALT_Z_ZETA_OMEGA_FRAC)
//#define STABILIZATION_ALTITUDE_NDI_SMALL_Z_OMEGA_2    BFP_OF_REAL((STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_OMEGA*STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_OMEGA), STAB_ALT_Z_OMEGA_2_FRAC)

struct FloatAltRefModel small_outer_ref_model = {
		STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_OMEGA, STABILIZATION_ALTITUDE_NDI_SMALL_Z_REF_ZETA
};

/* second order reference model natural frequency and damping (large angle inner reference model)*/
//#ifndef STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_OMEGA
//#define STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_OMEGA RadOfDeg(100.)
//#endif
//#ifndef STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_ZETA
//#define STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_ZETA  0.85
//#endif
//#define STABILIZATION_ALTITUDE_NDI_LARGE_ZD_ZETA_OMEGA BFP_OF_REAL((STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_ZETA*STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_OMEGA), STAB_ALT_ZD_ZETA_OMEGA_FRAC)
//#define STABILIZATION_ALTITUDE_NDI_LARGE_ZD_OMEGA_2    BFP_OF_REAL((STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_OMEGA*STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_OMEGA), STAB_ALT_ZD_OMEGA_2_FRAC)

struct FloatAltRefModel large_inner_ref_model = {
		STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_OMEGA, STABILIZATION_ALTITUDE_NDI_LARGE_ZD_REF_ZETA
};

/* second order reference model natural frequency and damping (large angle outer reference model)*/
//#ifndef STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_OMEGA
//#define STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_OMEGA RadOfDeg(100.)
//#endif
//#ifndef STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_ZETA
//#define STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_ZETA  0.85
//#endif
//#define STABILIZATION_ALTITUDE_NDI_LARGE_Z_ZETA_OMEGA BFP_OF_REAL((STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_ZETA*STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_OMEGA), STAB_ALT_Z_ZETA_OMEGA_FRAC)
//#define STABILIZATION_ALTITUDE_NDI_LARGE_Z_OMEGA_2    BFP_OF_REAL((STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_OMEGA*STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_OMEGA), STAB_ALT_Z_OMEGA_2_FRAC)

struct FloatAltRefModel large_outer_ref_model = {
		STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_OMEGA, STABILIZATION_ALTITUDE_NDI_LARGE_Z_REF_ZETA
};

/* adaptation */
#ifndef GUIDANCE_V_NOMINAL_HOVER_THROTTLE
#define GUIDANCE_V_NOMINAL_HOVER_THROTTLE 0.655;
#endif
#ifndef GUIDANCE_V_ADAPT_THROTTLE_ENABLED
#define GUIDANCE_V_ADAPT_THROTTLE_ENABLED FALSE;
#endif

/* flight mode transition */
#ifndef QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW
#define QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW RadOfDeg(30.)
#endif
#ifndef QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH
#define QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH RadOfDeg(45.)
#endif

#define ALT_GAINS_FRAC 7

/* Reference model state structs */
struct IntAltRefModelState small_inner_ref_model_state; // x = zd!!
struct IntAltRefModelState large_inner_ref_model_state; // x = zd!!
struct IntAltRefModelState small_outer_ref_model_state; // x = z!!
struct IntAltRefModelState large_outer_ref_model_state; // x = z!!

/* error integrator and file output (desired average thrust) */
int64_t small_z_sum_err;  //with INT64_STAB_ALT_ZD_FRAC
int64_t small_zd_sum_err;  //with INT64_STAB_ALT_ZDD_FRAC
int64_t large_z_sum_err;  //with INT64_STAB_ALT_ZD_FRAC
int64_t large_zd_sum_err;  //with INT64_STAB_ALT_ZDD_FRAC
int32_t thrust_avg;

/* local setpoints and virtual inputs */
int64_t small_zd_sp; //with INT64_STAB_ALT_ZD_FRAC
int64_t large_zd_sp; //with INT64_STAB_ALT_ZD_FRAC
int64_t small_zdd_fb; //virtual input for NDI part, with INT64_STAB_ALT_ZDD_FRAC
int64_t large_zdd_fb; //virtual input for NDI part, with INT64_STAB_ALT_ZDD_FRAC
int64_t small_zdd_ff; //virtual input for NDI part, with INT64_STAB_ALT_ZDD_FRAC
int64_t large_zdd_ff; //virtual input for NDI part, with INT64_STAB_ALT_ZDD_FRAC
int64_t small_zdd_sp; //virtual input for NDI part, with INT64_STAB_ALT_ZDD_FRAC
int64_t large_zdd_sp; //virtual input for NDI part, with INT64_STAB_ALT_ZDD_FRAC

/* extern variables, setpoints for attitude controller */
struct Int32Quat altitude_attitude_sp;
int32_t altitude_t_avg;
struct Int32Quat stab_att_sp_quat;

/* adaptation */
bool_t altitude_nominal_throttle = GUIDANCE_V_NOMINAL_HOVER_THROTTLE;
bool_t altitude_adapt_throttle_enabled = GUIDANCE_V_ADAPT_THROTTLE_ENABLED;

/* System parameters */
int32_t mass; // system mass, with INT32_STAB_ALT_MASS_FRAC

int32_t inert_x; // moment of inertia around x axis, with INT32_STAB_ALT_INERT_FRAC
int32_t inert_y; // moment of inertia around y axis, with INT32_STAB_ALT_INERT_FRAC
int32_t inert_z; // moment of inertia around z axis, with INT32_STAB_ALT_INERT_FRAC

/* Thrust settings */
int32_t small_tavg; // with INT32_STAB_ALT_T_FRAC
int32_t large_tavg; // with INT32_STAB_ALT_T_FRAC

/* desired tilt angles */
int32_t small_alpha; // with INT32_ANGLE_FRAC
int32_t large_alpha; // with INT32_ANGLE_FRAC

struct Int32Quat attitude_ref_small;
struct Int32Quat attitude_ref_large;

void stabilization_altitude_init(void) {

	/* reference models states equal to zero */
	small_inner_ref_model_state.stab_alt_x_ref = 0;
	small_inner_ref_model_state.stab_alt_xd_ref = 0;
	small_inner_ref_model_state.stab_alt_ff_ref = 0;
	small_inner_ref_model_state.stab_alt_xdd_ref = 0;

	small_outer_ref_model_state.stab_alt_x_ref = 0;
	small_outer_ref_model_state.stab_alt_xd_ref = 0;
	small_outer_ref_model_state.stab_alt_ff_ref = 0;
	small_outer_ref_model_state.stab_alt_xdd_ref = 0;

	large_inner_ref_model_state.stab_alt_x_ref = 0;
	large_inner_ref_model_state.stab_alt_xd_ref = 0;
	large_inner_ref_model_state.stab_alt_ff_ref = 0;
	large_inner_ref_model_state.stab_alt_xdd_ref = 0;

	large_outer_ref_model_state.stab_alt_x_ref = 0;
	large_outer_ref_model_state.stab_alt_xd_ref = 0;
	large_outer_ref_model_state.stab_alt_ff_ref = 0;
	large_outer_ref_model_state.stab_alt_xdd_ref = 0;

	small_z_sum_err = 0;
	small_zd_sum_err = 0;
	large_z_sum_err = 0;
	large_zd_sum_err = 0;

}

/*
 * Reference
 */
#define DT_UPDATE (1./PERIODIC_FREQUENCY)
// CAUTION! Periodic frequency is assumed to be 512 Hz
// which is equal to >> 9
#define F_UPDATE_RES 9

static void stabilization_altitude_update_ref(int64_t *x_sp, struct FloatAltRefModel *ref_model, struct IntAltRefModelState *ref_model_state, int64_t *v_h,bool_t enable_integrator) {

	int64_t zeta_omega;
	zeta_omega = BFP_OF_REAL((ref_model->omega*ref_model->zeta), STAB_ALT_ZD_ZETA_OMEGA_FRAC);
	int64_t omega_2;
	omega_2 = BFP_OF_REAL((ref_model->omega*ref_model->omega), STAB_ALT_ZD_OMEGA_2_FRAC);

	// compute xdd = -2*zeta*omega*xd -omega^2(x_sp - x)
	int64_t xdd_vel;
	xdd_vel = ((((int64_t)-2)*zeta_omega)*(ref_model_state->stab_alt_xd_ref>>(INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_XDD_REF_FRAC)))>>STAB_ALT_ZD_ZETA_OMEGA_FRAC;
	// compute zd error in zd_sp resolution
	int64_t x_err; // with INT64_STAB_ALT_ZD_REF_FRAC
	x_err = *x_sp - ref_model_state->stab_alt_x_ref;
	int64_t xdd_err;
	xdd_err = ((omega_2)*(x_err >> (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_XDD_REF_FRAC)))>>STAB_ALT_ZD_OMEGA_2_FRAC;
	ref_model_state->stab_alt_xdd_ref = xdd_vel + xdd_err;

	/* integrate xdd */
	int64_t delta_xd;
	delta_xd = (ref_model_state->stab_alt_xdd_ref) >> (F_UPDATE_RES + INT64_STAB_ALT_XDD_REF_FRAC - INT64_STAB_ALT_XD_REF_FRAC);
	ref_model_state->stab_alt_xd_ref += delta_xd;

	/*set feedforward term*/
	ref_model_state->stab_alt_ff_ref = ref_model_state->stab_alt_xd_ref;

	/*subtract pch v_h if in flight*/
	if (enable_integrator){
		ref_model_state->stab_alt_xd_ref = ref_model_state->stab_alt_xd_ref - *v_h;
	}

	/* integrate xd */
	int64_t delta_x;
	delta_x = (ref_model_state->stab_alt_xd_ref) >> (F_UPDATE_RES + INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_X_REF_FRAC);
	ref_model_state->stab_alt_x_ref += delta_x;

}

/* feedforward function inner loop*/
static void altitude_run_inner_ff(int64_t *inner_zdd_ff, struct IntAltRefModelState *inner_ref_model_state, struct Int32NDIAltitudeGains *gains)
{
	/* Compute feedforward based on reference acceleration */
	*inner_zdd_ff = (((int64_t)gains->ff)*inner_ref_model_state->stab_alt_ff_ref >> (INT64_STAB_ALT_XD_REF_FRAC- INT64_STAB_ALT_ZDD_FRAC));

}

/* feedforward function outer loop*/
static void altitude_run_outer_ff(int64_t *outer_zd_ff, struct IntAltRefModelState *outer_ref_model_state, struct Int32NDIAltitudeGains *gains)
{
	/* Compute feedforward based on reference acceleration */
	*outer_zd_ff = (((int64_t)gains->ff)*outer_ref_model_state->stab_alt_ff_ref >> (INT64_STAB_ALT_XD_REF_FRAC- INT64_STAB_ALT_ZD_FRAC));

}

/* PID feedback function */
static void altitude_run_fb(struct Int32NDIAltitudeGains *gains, int64_t *err, int64_t *sum_err, int64_t *der_err, int64_t *input)
{
	/*  PID feedback */
	*input = INT_MULT_RSHIFT(((int64_t)gains->p), *err, ALT_GAINS_FRAC) + INT_MULT_RSHIFT(((int64_t)gains->i), *sum_err,ALT_GAINS_FRAC) + INT_MULT_RSHIFT(((int64_t)gains->d), *der_err,ALT_GAINS_FRAC);

}

static void altitude_calc_mass(void){

	/*TODO: invert thrust relation with COMMAND_THRUST*/

	/* Find average thrust setting */
	int32_t inv_m; // with GV_ADAPT_X_FRAC
	if (altitude_adapt_throttle_enabled) {
		inv_m =  gv_adapt_X;
	}
	else {
		/* use the fixed nominal throttle */
		inv_m = BFP_OF_REAL(9.81 / (guidance_v_nominal_throttle * MAX_PPRZ), GV_ADAPT_X_FRAC);
	}

	/*TODO: INT*/
  int32_t thrust; // with INT32_STAB_ALT_T_FRAC
  int16_t thrust_cmd = (int16_t)(9.81/FLOAT_OF_BFP(inv_m,GV_ADAPT_X_FRAC));
  attitude_t_from_tcommand(&thrust,&thrust_cmd);

//  mass = ((int32_t)(thrust*4./9.80665)) << (INT32_STAB_ALT_MASS_FRAC - INT32_STAB_ALT_T_FRAC);
//
//  alt_test1 = FLOAT_OF_BFP(mass,INT32_STAB_ALT_MASS_FRAC);

  /*DEBUG REMOVE altitude tuning fixed mass*/
  mass = BFP_OF_REAL(0.45,INT32_STAB_ALT_MASS_FRAC);

}

/*
 * Small angle controller outer loop, controls altitude, calculates desired vertical velocity
 */
static void altitude_run_small_outer(bool_t enable_integrator){

	/* calculate error, integrated error, error derivative */
	int64_t small_outer_z_err; // INT64_STAB_ALT_ZD_FRAC
	small_outer_z_err = (small_outer_ref_model_state.stab_alt_x_ref >> (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_ZD_FRAC)) - ((int64_t)(stateGetPositionNed_i()->z) << (INT64_STAB_ALT_ZD_FRAC - INT32_POS_FRAC));

	if (enable_integrator && (FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC) < QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH))
		small_z_sum_err += small_outer_z_err >> (F_UPDATE_RES);
	else
		small_z_sum_err = 0;

	/*limit integrator terms (just in case, shouldn't be neccesary due to PCH)*/
	if (small_z_sum_err > (((int64_t)1) << (54 - INT64_STAB_ALT_ZD_FRAC)))
		small_z_sum_err = (((int64_t)1) << (54 - INT64_STAB_ALT_ZD_FRAC));
	else if (small_z_sum_err < -(((int64_t)1) << (54 - INT64_STAB_ALT_ZD_FRAC)))
		small_z_sum_err = -(((int64_t)1) << (54 - INT64_STAB_ALT_ZD_FRAC));

	int64_t small_outer_zd_err; // with INT64_STAB_ALT_ZD_FRAC
	small_outer_zd_err = ((small_outer_ref_model_state.stab_alt_xd_ref >> (INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_ZD_FRAC)) - (int64_t)((stateGetSpeedNed_i()->z) << (INT64_STAB_ALT_ZD_FRAC - INT32_SPEED_FRAC)));

	/* run ff and fb loop */
	int64_t small_zd_fb;
	altitude_run_fb(&small_outer_gains, &small_outer_z_err, &small_z_sum_err, &small_outer_zd_err, &small_zd_fb);

	int64_t small_zd_ff;
	altitude_run_outer_ff(&small_zd_ff,&small_outer_ref_model_state,&small_outer_gains);

	small_zd_sp = small_zd_fb + small_zd_ff;

}

/*
 * large angle controller outer loop, controls altitude, calculates desired vertical velocity
 */
static void altitude_run_large_outer(bool_t enable_integrator){

	/* calculate error, integrated error, error derivative */
	int64_t large_outer_z_err; // with INT64_STAB_ALT_ZD_FRAC
	large_outer_z_err = (large_outer_ref_model_state.stab_alt_x_ref >> (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_ZD_FRAC)) - ((int64_t)(stateGetPositionNed_i()->z) << (INT64_STAB_ALT_ZD_FRAC - INT32_POS_FRAC));

	if (enable_integrator && (FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC) > QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW))
		large_z_sum_err += large_outer_z_err >> (F_UPDATE_RES);
	else
		large_z_sum_err = 0;

	/*limit integrator terms (just in case, shouldn't be neccesary due to PCH)*/
	if (large_z_sum_err > (((int64_t)1) << (54 - INT64_STAB_ALT_ZD_FRAC)))
		large_z_sum_err = (((int64_t)1) << (54 - INT64_STAB_ALT_ZD_FRAC));
	else if (large_z_sum_err < -(((int64_t)1) << (54 - INT64_STAB_ALT_ZD_FRAC)))
		large_z_sum_err = -(((int64_t)1) << (54 - INT64_STAB_ALT_ZD_FRAC));

	int64_t large_outer_zd_err; // with INT64_STAB_ALT_ZD_FRAC
	large_outer_zd_err = (large_outer_ref_model_state.stab_alt_xd_ref >> (INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_ZD_FRAC)) - (int64_t)((stateGetSpeedNed_i()->z) << (INT64_STAB_ALT_ZD_FRAC - INT32_SPEED_FRAC));

	/* run ff and fb loop */
	int64_t large_zd_fb;
	altitude_run_fb(&large_outer_gains, &large_outer_z_err, &large_z_sum_err, &large_outer_zd_err, &large_zd_fb);

	int64_t large_zd_ff;
	altitude_run_outer_ff(&large_zd_ff,&large_outer_ref_model_state,&large_outer_gains);

	large_zd_sp = large_zd_fb + large_zd_ff;

}

/*
 * Small angle controller inner loop, controls vertical speed, calculates desired tilt angle and thrust
 */
static void altitude_run_small_inner(bool_t enable_integrator){

	/* calculate error, integrated error, error derivative */
	int64_t small_inner_zd_err; // with INT64_STAB_ALT_ZDD_FRAC
	small_inner_zd_err = (small_inner_ref_model_state.stab_alt_x_ref >> (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_ZDD_FRAC)) - (((int64_t)stateGetSpeedNed_i()->z) >> (INT32_SPEED_FRAC - INT64_STAB_ALT_ZDD_FRAC));

	if (enable_integrator && (FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC) < QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH))
		small_zd_sum_err += small_inner_zd_err >> (F_UPDATE_RES);
	else
		small_zd_sum_err = 0;

	/*limit integrator terms (just in case, shouldn't be neccesary due to PCH)*/
	if (small_zd_sum_err > (((int64_t)1) << (54 - INT64_STAB_ALT_ZDD_FRAC)))
		small_zd_sum_err = (((int64_t)1) << (54 - INT64_STAB_ALT_ZDD_FRAC));
	else if (small_zd_sum_err < -(((int64_t)1) << (54 - INT64_STAB_ALT_ZDD_FRAC)))
		small_zd_sum_err = -(((int64_t)1) << (54 - INT64_STAB_ALT_ZDD_FRAC));

	int64_t small_inner_zdd_err; // with INT64_STAB_ALT_ZDD_FRAC
	small_inner_zdd_err = (((small_inner_ref_model_state.stab_alt_xd_ref >> (INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_ZDD_FRAC))) - ((((int64_t)stateGetAccelNed_i()->z) << (INT64_STAB_ALT_ZDD_FRAC - INT32_ACCEL_FRAC))));

	/* run feedback loop */
	altitude_run_fb(&small_inner_gains, &small_inner_zd_err, &small_zd_sum_err, &small_inner_zdd_err, &small_zdd_fb);

	/* run feedforward loop */
  altitude_run_inner_ff(&small_zdd_ff, &small_inner_ref_model_state, &small_inner_gains);

  /* sum fb and ff */
  small_zdd_sp = small_zdd_fb + small_zdd_ff;

  /*find tavg*/
  int64_t g_m_zdd_small; // with INT64_STAB_ALT_ZDD_FRAC
  g_m_zdd_small = BFP_OF_REAL(9.80665, INT64_STAB_ALT_ZDD_FRAC) - (small_zdd_sp);

  int64_t Ktilt; // tilt correction for vertical thrust based on state, with INT64_STAB_ALT_ZDD_FRAC
  Ktilt = ((((int64_t)1)*(1<<(2*15)))/
  		((int64_t)INT_MULT_RSHIFT(stateGetNedToBodyQuat_i()->qi,stateGetNedToBodyQuat_i()->qi,INT32_QUAT_FRAC) -
  				(int64_t)INT_MULT_RSHIFT(stateGetNedToBodyQuat_i()->qx,stateGetNedToBodyQuat_i()->qx,INT32_QUAT_FRAC) -
  				(int64_t)INT_MULT_RSHIFT(stateGetNedToBodyQuat_i()->qy,stateGetNedToBodyQuat_i()->qy,INT32_QUAT_FRAC) +
					(int64_t)INT_MULT_RSHIFT(stateGetNedToBodyQuat_i()->qz,stateGetNedToBodyQuat_i()->qz,INT32_QUAT_FRAC)))
					>> (INT32_QUAT_FRAC - INT64_STAB_ALT_ZDD_FRAC);

	small_tavg = (int32_t)(((INT_MULT_RSHIFT(g_m_zdd_small,Ktilt,INT64_STAB_ALT_ZDD_FRAC)*mass) >> (INT32_STAB_ALT_MASS_FRAC + INT64_STAB_ALT_ZDD_FRAC - INT32_STAB_ALT_T_FRAC))/4);

	/* limit between 0 and max thrust */
	if (small_tavg < 0){
		small_tavg = 0;
	}
	else if (small_tavg > getMaxTavg()){
		small_tavg = getMaxTavg();
	}

	/* small angle controller desired tilt angle is equal to reference tilt angle */
	int32_t arg_acos;
	arg_acos = (1<<2*INT32_QUAT_FRAC) - 2*(stab_att_sp_quat.qx*stab_att_sp_quat.qx + stab_att_sp_quat.qy*stab_att_sp_quat.qy);

	/* TODO: acos in BFP */
	float small_alpha_f;
	small_alpha_f = acosf(QUAT1_FLOAT_OF_BFP(QUAT1_FLOAT_OF_BFP(arg_acos)));
	small_alpha = ANGLE_BFP_OF_REAL(small_alpha_f);

}

/*
 * Large angle controller inner loop, controls vertical speed, calculates desired tilt angle and thrust
 */
static void altitude_run_large_inner(bool_t enable_integrator){

	/* thrust setting based on attitude reference tilt angle TODO: INT COS and ACOS */
	float large_alpha_ref_f;
	float large_tavg_arg_acos_f;
	float large_tavg_f;
	large_tavg_arg_acos_f = (1<<2*INT32_QUAT_FRAC) - 2*(stab_att_sp_quat.qx*stab_att_sp_quat.qx + stab_att_sp_quat.qy*stab_att_sp_quat.qy);
	large_alpha_ref_f = acosf(QUAT1_FLOAT_OF_BFP(QUAT1_FLOAT_OF_BFP(large_tavg_arg_acos_f)));

	if (cosf(large_alpha_ref_f) == 0){
		large_tavg = getMaxTavg();
	  large_tavg_f = FLOAT_OF_BFP(large_tavg,INT32_STAB_ALT_T_FRAC);
	}
	else {
		/*calculate tavg based on desired tilt angle, add correction for thrust reduction with speed (0.25*desired tilt angle)*/
		large_tavg_f = (FLOAT_OF_BFP(mass,INT32_STAB_ALT_MASS_FRAC))*9.80665/(cosf(large_alpha_ref_f)*4.) + 0.7*large_alpha_ref_f;
		large_tavg = BFP_OF_REAL(large_tavg_f,INT32_STAB_ALT_T_FRAC);
	}

	/* limit between 0 and max thrust */
	if (large_tavg < 0){
		large_tavg = 0;
	}
	else if (large_tavg > getMaxTavg()){
		large_tavg = getMaxTavg();
	}

	/* calculate error, integrated error, error derivative */
	int64_t large_inner_zd_err; // with INT64_STAB_ALT_ZDD_FRAC
	large_inner_zd_err = (large_inner_ref_model_state.stab_alt_x_ref >> (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_ZDD_FRAC)) - (((int64_t)(stateGetSpeedNed_i()->z) >> (INT32_SPEED_FRAC - INT64_STAB_ALT_ZDD_FRAC)));

	if (enable_integrator && (FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC) > QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW))
		large_zd_sum_err += large_inner_zd_err >> (F_UPDATE_RES);
	else
		large_zd_sum_err = 0;

	/*limit integrator terms (just in case, shouldn't be neccesary due to PCH)*/
	if (large_zd_sum_err > (((int64_t)1) << (54 - INT64_STAB_ALT_ZDD_FRAC)))
		large_zd_sum_err = (((int64_t)1) << (54 - INT64_STAB_ALT_ZDD_FRAC));
	else if (large_zd_sum_err < -(((int64_t)1) << (54 - INT64_STAB_ALT_ZDD_FRAC)))
		large_zd_sum_err = -(((int64_t)1) << (54 - INT64_STAB_ALT_ZDD_FRAC));

	int64_t large_inner_zdd_err; // with INT64_STAB_ALT_ZDD_FRAC
	large_inner_zdd_err = ((large_inner_ref_model_state.stab_alt_xd_ref >> (INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_ZDD_FRAC))) - (((int64_t)(stateGetAccelNed_i()->z) << (INT64_STAB_ALT_ZDD_FRAC - INT32_ACCEL_FRAC)));

	/* run feedback loop */
	altitude_run_fb(&large_inner_gains, &large_inner_zd_err, &large_zd_sum_err, &large_inner_zdd_err, &large_zdd_fb);

	/* run feedforward loop */
	altitude_run_inner_ff(&large_zdd_ff, &large_inner_ref_model_state, &large_inner_gains);

	/* sum fb and ff */
	large_zdd_sp = large_zdd_fb + large_zdd_ff;

	/* invert sign and add gravity */
  int64_t g_m_zdd_large; // with INT64_STAB_ALT_ZDD_FRAC in negative z earth direction!
  g_m_zdd_large = BFP_OF_REAL(9.80665,INT64_STAB_ALT_ZDD_FRAC) - large_zdd_sp;

	/* calculate large angle controller desired tilt angle based commanded average thrust and desired zdd sp TODO: INT ACOS */
  float large_alpha_f;
  float g_m_zdd_large_f;
  float mass_f;
  float large_t_total_f;
  g_m_zdd_large_f = FLOAT_OF_BFP(g_m_zdd_large,INT64_STAB_ALT_ZDD_FRAC);

  mass_f = FLOAT_OF_BFP(mass,INT32_STAB_ALT_MASS_FRAC);
  large_t_total_f = FLOAT_OF_BFP(large_tavg*4, INT32_STAB_ALT_T_FRAC);

  /*Prevent acosf arguments outside range -1 to 1 and division by zero*/
  if (large_t_total_f == 0)
  	large_alpha_f = 0;
  else if (g_m_zdd_large_f*mass_f < -large_t_total_f)
  	large_alpha_f = 3.14;
  else if (g_m_zdd_large_f*mass_f > large_t_total_f)
  	large_alpha_f = 0;
  else
  	large_alpha_f = acosf(g_m_zdd_large_f*mass_f/(large_t_total_f));

  /*Limit alpha between -80 and 80 deg */
  if (large_alpha_f > 80./180.*3.14){
  	large_alpha_f = 80./180.*3.14;
  }
  else if (large_alpha_f < -80./180.*3.14){
  	large_alpha_f = -80./180.*3.14;
  }

  large_alpha = ANGLE_BFP_OF_REAL(large_alpha_f);

}

void stabilization_altitude_run(bool_t enable_integrator) {

	/*DEBUG REMOVE*/
	if (alt_sp && autopilot_mode == AP_MODE_TUNE_NDI && stabilization_override_on){
		altitude_z_sp = ((-z_sp)*((int64_t)1<<(INT64_STAB_ALT_X_REF_FRAC)));

		time_counter_tmp = time_counter_tmp + 1;

		if (time_counter_tmp > 1100){
			altitude_z_sp = ((-5.5)*((int64_t)1<<(INT64_STAB_ALT_X_REF_FRAC)));
		}
	}
	else{
		time_counter_tmp = 0;
	}

	/*DEBUG REMOVE*/
	alt_test1 = ((float)(altitude_z_sp)/((int64_t)1<<(INT64_STAB_ALT_X_REF_FRAC)));

	/*find PCH correction v_h altitude*/
	int64_t small_v_h_z; // INT64_STAB_ALT_XD_REF_FRAC
	int64_t large_v_h_z; // INT64_STAB_ALT_XD_REF_FRAC
	small_v_h_z = (small_zd_sp << (INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_ZD_FRAC)) - (small_inner_ref_model_state.stab_alt_x_ref >> (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_XD_REF_FRAC));
	large_v_h_z = (large_zd_sp << (INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_ZD_FRAC)) - (large_inner_ref_model_state.stab_alt_x_ref >> (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_XD_REF_FRAC));

//	/*DEBUG REMOVE*/
//	alt_test4 = FLOAT_OF_BFP(large_v_h_z,INT64_STAB_ALT_XD_REF_FRAC);
////
//	large_v_h_z = 0;

	/* run altitude reference models */
	stabilization_altitude_update_ref(&altitude_z_sp, &small_outer_ref_model, &small_outer_ref_model_state,&small_v_h_z,enable_integrator);
	stabilization_altitude_update_ref(&altitude_z_sp, &large_outer_ref_model, &large_outer_ref_model_state,&large_v_h_z,enable_integrator);

//	/*DEBUG REMOVE*/
//	alt_test4 = FLOAT_OF_BFP(large_outer_ref_model_state.stab_alt_ff_ref,INT64_STAB_ALT_XD_REF_FRAC);

	/*DEBUG REMOVE*/
	alt_test2 = ((float)(small_outer_ref_model_state.stab_alt_x_ref)/((int64_t)1<<(INT64_STAB_ALT_X_REF_FRAC)));

	/*
	 * if in VERTICAL_MODE_CLIMB don't use outer loop
	 */

	if (altitude_vert_mode == 1 || (alt_d_sp && autopilot_mode == AP_MODE_TUNE_NDI && stabilization_override_on)){

		small_zd_sp = altitude_zd_sp >> (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_ZD_FRAC);
		large_zd_sp = altitude_zd_sp >> (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_ZD_FRAC);

	}
	else {
		/* run outer loop of small angle controller, calculates vertical velocity set point small_zd_sp */
		altitude_run_small_outer(enable_integrator);

		/* run outer loop of large angle controller, calculates vertical velocity set point large_zd_sp*/
		altitude_run_large_outer(enable_integrator);
	}

	/*DEBUG REMOVE*/
	if (alt_d_sp && autopilot_mode == AP_MODE_TUNE_NDI && stabilization_override_on){
		small_zd_sp = ((-z_d_sp)*((int64_t)1<<(INT64_STAB_ALT_ZD_FRAC)));
		large_zd_sp = ((-z_d_sp)*((int64_t)1<<(INT64_STAB_ALT_ZD_FRAC)));
	}

	/*DEBUG REMOVE*/
	alt_test3 = ((float)(small_zd_sp)/((int64_t)1<<(INT64_STAB_ALT_ZD_FRAC)));

	/*find PCH correction v_h vert. vel. DEBUG VH LARGE*/
	int64_t small_v_h_zd; // INT64_STAB_ALT_XD_REF_FRAC
	int64_t large_v_h_zd; // INT64_STAB_ALT_XD_REF_FRAC
	small_v_h_zd = (small_zdd_sp << (INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_XDD_REF_FRAC)) - ((int64_t)pch_trans_accel.z << (INT64_STAB_ALT_XD_REF_FRAC - INT32_PCH_F_FRAC));
	large_v_h_zd = (large_zdd_sp << (INT64_STAB_ALT_XD_REF_FRAC - INT64_STAB_ALT_XDD_REF_FRAC)) - ((int64_t)pch_trans_accel.z << (INT64_STAB_ALT_XD_REF_FRAC - INT32_PCH_F_FRAC));

	/* run vertical velocity reference models */
  int64_t small_zd_sp_ref_scale; //setpoint in reference model frac, with INT64_STAB_ALT_X_REF_FRAC
  int64_t large_zd_sp_ref_scale; //setpoint in reference model frac, with INT64_STAB_ALT_X_REF_FRAC
  small_zd_sp_ref_scale = small_zd_sp << (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_ZD_FRAC);
  large_zd_sp_ref_scale = large_zd_sp << (INT64_STAB_ALT_X_REF_FRAC - INT64_STAB_ALT_ZD_FRAC);

	stabilization_altitude_update_ref(&small_zd_sp_ref_scale, &small_inner_ref_model, &small_inner_ref_model_state, &small_v_h_zd,enable_integrator);
	stabilization_altitude_update_ref(&large_zd_sp_ref_scale, &large_inner_ref_model, &large_inner_ref_model_state, &large_v_h_zd,enable_integrator);

	/*DEBUG REMOVE*/
	alt_test4 = ((float)(small_inner_ref_model_state.stab_alt_x_ref)/((int64_t)1<<(INT64_STAB_ALT_X_REF_FRAC)));

	/* calculate mass */
	altitude_calc_mass();

	/* run inner loop of small angle controller, calculates thrust setting small_tavg and tilt angle small_alpha */
	altitude_run_small_inner(enable_integrator);

	/* run inner loop of large angle controller, calculates thrust setting large_tavg and tilt angle large_alpha */
	altitude_run_large_inner(enable_integrator);

  /*
   * Mix tilt angles and thrust settings of both controllers based on flight mode transition law,
   * a linear switch between QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW and QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH is used
   */
  int32_t alpha_des; //with INT32_ANGLE_FRAC

	if (FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC) < QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW) {
		alpha_des = small_alpha;
	  altitude_t_avg = small_tavg;
	}
	else if (FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC) > QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH) {
		alpha_des = large_alpha;
		altitude_t_avg = large_tavg;
	}
	else {
		alpha_des = (int32_t)(1./(QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH - QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW)*
				(FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC) - QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW)*large_alpha +
				1./(QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH - QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW)*
				(QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH - FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC))*small_alpha);

		altitude_t_avg = (int32_t)(1./(QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH - QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW)*
				(FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC) - QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW)*large_tavg +
				1./(QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH - QUAT_FLIGHT_MODE_TRANSITION_LIMIT_LOW)*
				(QUAT_FLIGHT_MODE_TRANSITION_LIMIT_HIGH - FLOAT_OF_BFP(small_alpha, INT32_ANGLE_FRAC))*small_tavg);
	}

	/*Limit thrust to joystick throttle*/
	int16_t t_avg_cmd;
	attitude_tcommand_from_t(&t_avg_cmd,&altitude_t_avg);
#if NO_RC_THRUST_LIMIT
#else
	t_avg_cmd = Min(guidance_v_rc_delta_t, t_avg_cmd);
	attitude_t_from_tcommand(&altitude_t_avg,&t_avg_cmd);
#endif

  /*
   * Calculate desired attitude using desired tilt angle and stick tilt direction in quaternions TODO: INT
   */

	/* desired alpha sine and cosine */
	float alpha_des_f, salpha_des_2_f;
	int32_t salpha_des_2; // with INT32_TRIG_FRAC
	alpha_des_f = FLOAT_OF_BFP(alpha_des,INT32_ANGLE_FRAC);
	salpha_des_2_f = sinf(alpha_des_f/2);
	salpha_des_2 = TRIG_BFP_OF_REAL(salpha_des_2_f);

	/* Find quat_ref alpha */
	int64_t arg_acos_ref;
	arg_acos_ref = (1<<2*INT32_QUAT_FRAC) - 2*(stab_att_sp_quat.qx*stab_att_sp_quat.qx + stab_att_sp_quat.qy*stab_att_sp_quat.qy);

	float alpha_ref_f;
	alpha_ref_f = acosf(QUAT1_FLOAT_OF_BFP(QUAT1_FLOAT_OF_BFP(arg_acos_ref)));

	/* find reference rx and ry */
  int32_t cpsi_ref_2, spsi_ref_2, salpha_ref_2; // with INT32_TRIG_FRAC
  int32_t rx, ry; // with INT32_TRIG_FRAC
  float salpha_ref_2_f, q3_ref_f, q0_ref_f, psi_ref_f, cpsi_ref_2_f, spsi_ref_2_f, rx_f, ry_f;

	q3_ref_f = QUAT1_FLOAT_OF_BFP(stab_att_sp_quat.qz);
	q0_ref_f = QUAT1_FLOAT_OF_BFP(stab_att_sp_quat.qi);

  salpha_ref_2_f = sinf(alpha_ref_f/2);
  salpha_ref_2 = TRIG_BFP_OF_REAL(salpha_ref_2_f);
	psi_ref_f = 2*atan2f(q3_ref_f,q0_ref_f);

	cpsi_ref_2_f = cosf(psi_ref_f/2);
	cpsi_ref_2 = TRIG_BFP_OF_REAL(cpsi_ref_2_f);
	spsi_ref_2_f = sinf(psi_ref_f/2);
	spsi_ref_2 = TRIG_BFP_OF_REAL(spsi_ref_2_f);

  if (salpha_ref_2_f == 0) {
  	rx = BFP_OF_REAL(1,INT32_TRIG_FRAC);
  	ry = 0;
  }
  else {

  	rx_f =
  			TRIG_FLOAT_OF_BFP((INT_MULT_RSHIFT(cpsi_ref_2,stab_att_sp_quat.qx,INT32_QUAT_FRAC) -
  					INT_MULT_RSHIFT(spsi_ref_2,stab_att_sp_quat.qy,INT32_QUAT_FRAC)))/TRIG_FLOAT_OF_BFP(salpha_ref_2);

  	ry_f =
  			TRIG_FLOAT_OF_BFP((INT_MULT_RSHIFT(spsi_ref_2,stab_att_sp_quat.qx,INT32_QUAT_FRAC) +
  					INT_MULT_RSHIFT(cpsi_ref_2,stab_att_sp_quat.qy,INT32_QUAT_FRAC)))/TRIG_FLOAT_OF_BFP(salpha_ref_2);
  	rx = TRIG_BFP_OF_REAL(rx_f);
  	ry = TRIG_BFP_OF_REAL(ry_f);
  }

  struct Int32Quat q_h;
  struct Int32Quat q_v;
  q_h.qi = QUAT1_BFP_OF_REAL(cosf(alpha_des_f/2));
  q_h.qx = INT_MULT_RSHIFT(salpha_des_2,rx,INT32_TRIG_FRAC) << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);
  q_h.qy = INT_MULT_RSHIFT(salpha_des_2,ry,INT32_TRIG_FRAC) << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);
  q_h.qz = 0;

  q_v.qi = cpsi_ref_2 << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);
  q_v.qx = 0;
  q_v.qy = 0;
  q_v.qz = spsi_ref_2 << (INT32_QUAT_FRAC - INT32_TRIG_FRAC);

  INT32_QUAT_COMP(altitude_attitude_sp, q_h, q_v);
  INT32_QUAT_NORMALIZE(altitude_attitude_sp);

//  /*DEBUG REMOVE*/
//  alt_test3 = alpha_des_f;

}
