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

/** @file stabilization_attitude_ndi_quat_int.c
 * Rotorcraft quaternion attitude stabilization with NDI control
 */

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"
#include "stabilization_attitude_ndi_quat_int.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "stabilization_attitude_ndi_pch_model.h"
#include "subsystems/electrical.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"
#include "paparazzi.h"

#include <math.h>
#include "std.h"
#include <stdlib.h>

int32_t vsupply_rest; //with INT32_VREST_FRAC

/*DEBUG REMOVE*/
#include "subsystems/radio_control.h"
#include "mcu_periph/sys_time.h"

/* DEBUG VARIABLE, REMOVE */
float test1;
float test2;
float test3;
float test4;
float test5;
float test6;
float test7;
float test8;
float test9;
float test10;
float test11;
float test12;
float test13;
float test14;
float test15;
float test16;
float test17;
float test18;
float test19;
bool_t stabilization_override_on = STABILIZATION_ATTITUDE_NDI_OVERRIDE_ON;
float phi_sp = STABILIZATION_ATTITUDE_NDI_PHI_SP;
float theta_sp = STABILIZATION_ATTITUDE_NDI_THETA_SP;
float psi_sp = STABILIZATION_ATTITUDE_NDI_PSI_SP;
float phi_d_sp = STABILIZATION_ATTITUDE_NDI_PHI_D_SP;
float theta_d_sp = STABILIZATION_ATTITUDE_NDI_THETA_D_SP;
float psi_d_sp = STABILIZATION_ATTITUDE_NDI_PSI_D_SP;
float z_sp = STABILIZATION_ATTITUDE_NDI_Z_SP;
float z_d_sp = STABILIZATION_ATTITUDE_NDI_Z_D_SP;
float tdiff_yaw_sp = STABILIZATION_ATTITUDE_NDI_TDIFF_SP;
bool_t att_sp = STABILIZATION_ATTITUDE_NDI_ATT_SP;
bool_t att_d_sp = STABILIZATION_ATTITUDE_NDI_ATT_D_SP;
bool_t alt_sp = STABILIZATION_ATTITUDE_NDI_ALT_SP;
bool_t alt_d_sp = STABILIZATION_ATTITUDE_NDI_ALT_D_SP;
bool_t tau_step = STABILIZATION_ATTITUDE_NDI_TAU;
struct Int32Quat quat_override_sp;
struct Int32Vect2 phi_theta_override_sp;
int32_t psi_override_sp;

/*DEBUG REMOVE RPM TESTS*/
int8_t time_counter;
uint32_t start_time;
int8_t step;
uint32_t ticks;
uint32_t start_ticks;

struct Int32NDIAttitudeGains attitude_ndi_gains = {
	{STABILIZATION_ATTITUDE_NDI_TILT_PGAIN, STABILIZATION_ATTITUDE_NDI_YAW_PGAIN},
	{STABILIZATION_ATTITUDE_NDI_TILT_DGAIN,STABILIZATION_ATTITUDE_NDI_YAW_DGAIN}
};

/* warn if some gains are still negative */
#if (STABILIZATION_ATTITUDE_NDI_TILT_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_NDI_YAW_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_NDI_TILT_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_NDI_YAW_DGAIN < 0)
#warning "ALL control gains are now positive!!!"
#endif

struct Int32Quat stabilization_att_sum_err_quat;
struct Int32Eulers stabilization_att_sum_err;
struct Int16Thrust attitude_thrust_command; // thrust command calculated by controller, limited by PPRZ_MAX
struct Int32Thrust attitude_thrust; // with INT32_STAB_ALT_T_FRAC
int16_t thrust_command[4]; // thrust command send to engines
struct Int32Rates  stab_rate_sp_fb;          //< with INT32_RATE_FRAC
struct Int32Rates  stab_rate_sp_ff;          //< with INT32_RATE_FRAC
struct Int32Rates  v_h_att; // with INT32_RATE_FRAC

/*thrust and rpm constants int and float*/
float c_rpm_f 		= 181.47;
float c_rpm_cmd_f = 0.05229;
float c_t_f 			= 0.0495;
float c_t_rpm_f 	= 0.0005651;
float c_t_rpm_2_f = 0.000005149;

#define INT32_C_RPM_FRAC 8
#define INT32_C_RPM_CMD_FRAC 19
#define INT32_C_T_FRAC 14
#define INT32_C_T_RPM_FRAC 24
#define INT32_C_T_RPM_2_FRAC 30

int32_t c_rpm = BFP_OF_REAL(181.47,INT32_C_RPM_FRAC);
int32_t c_rpm_cmd = BFP_OF_REAL(0.05229,INT32_C_RPM_CMD_FRAC);
int32_t c_t = BFP_OF_REAL(0.0495,INT32_C_T_FRAC);
int32_t c_t_rpm = BFP_OF_REAL(0.0005651,INT32_C_T_RPM_FRAC);
int32_t c_t_rpm_2 = BFP_OF_REAL(0.000005149,INT32_C_T_RPM_2_FRAC);

/* extern variables */
int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];
int32_t attitude_t_avg_cmd = 0;
float psi_f;

//struct Int32Rates stab_rate_sp; // rate setpoint commanded by attitude controller
struct Int32ThrustDiff rate_thrust_diff; // thrust differences commanded by rate controller, INT32_STAB_ALT_T_FRAC

#define IERROR_SCALE 1
#define GAIN_PRESCALER_FF 1
#define GAIN_PRESCALER_P 1
#define GAIN_PRESCALER_D 1
#define GAIN_PRESCALER_I 1

#define ATT_GAINS_FRAC 7
#define INT64_C_BAT_FRAC 40
#define INT32_VREST_FRAC 12
#define INT32_RPM_FRAC 8

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_att(void) { //FIXME really use this message here ?
  struct Int32Rates* body_rate = stateGetBodyRates_i();
  struct Int32Eulers* att = stateGetNedToBodyEulers_i();
  DOWNLINK_SEND_STAB_ATTITUDE_INT(DefaultChannel, DefaultDevice,
      &(body_rate->p), &(body_rate->q), &(body_rate->r),
      &(att->phi), &(att->theta), &(att->psi),
      &stab_att_sp_euler.phi,
      &stab_att_sp_euler.theta,
      &stab_att_sp_euler.psi,
      &stabilization_att_sum_err.phi,
      &stabilization_att_sum_err.theta,
      &stabilization_att_sum_err.psi,
      &stabilization_att_fb_cmd[COMMAND_ROLL],
      &stabilization_att_fb_cmd[COMMAND_PITCH],
      &stabilization_att_fb_cmd[COMMAND_YAW],
      &stabilization_att_ff_cmd[COMMAND_ROLL],
      &stabilization_att_ff_cmd[COMMAND_PITCH],
      &stabilization_att_ff_cmd[COMMAND_YAW],
      &stabilization_cmd[COMMAND_ROLL],
      &stabilization_cmd[COMMAND_PITCH],
      &stabilization_cmd[COMMAND_YAW]);
}

static void send_att_ref(void) {
  DOWNLINK_SEND_STAB_ATTITUDE_REF_INT(DefaultChannel, DefaultDevice,
                                      &stab_att_sp_euler.phi,
                                      &stab_att_sp_euler.theta,
                                      &stab_att_sp_euler.psi,
                                      &stab_att_ref_euler.phi,
                                      &stab_att_ref_euler.theta,
                                      &stab_att_ref_euler.psi,
                                      &stab_att_ref_rate.p,
                                      &stab_att_ref_rate.q,
                                      &stab_att_ref_rate.r,
                                      &stab_att_ref_accel.p,
                                      &stab_att_ref_accel.q,
                                      &stab_att_ref_accel.r);
}

/* debug messages REMOVE*/
static void send_control_test(void) {
  DOWNLINK_SEND_CONTROL_TEST(DefaultChannel, DefaultDevice,
                                      &test1,
                                      &test2,
                                      &test3,
                                      &test4,
                                      &test5,
                                      &test6,
                                      &test7,
                                      &test8,
                                      &test9,
                                      &test10,
                                      &test11,
                                      &test12,
                                      &test13,
                                      &test14,
                                      &test15,
                                      &test16,
                                      &test17,
                                      &test18,
                                      &test19
                                      );
}


static void send_ahrs_ref_quat(void) {
  struct Int32Quat* quat = stateGetNedToBodyQuat_i();
  DOWNLINK_SEND_AHRS_REF_QUAT(DefaultChannel, DefaultDevice,
      &stab_att_ref_quat.qi,
      &stab_att_ref_quat.qx,
      &stab_att_ref_quat.qy,
      &stab_att_ref_quat.qz,
      &(quat->qi),
      &(quat->qx),
      &(quat->qy),
      &(quat->qz));
}
#endif

void stabilization_attitude_init(void) {

  stabilization_attitude_ref_init();
  stabilization_altitude_init();
  stabilization_rate_ndi_init();
  stabilization_pch_init();

  INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
  INT_EULERS_ZERO( stabilization_att_sum_err );

  thrust_command[0] = 0;
  thrust_command[1] = 0;
  thrust_command[2] = 0;
  thrust_command[3] = 0;

  /*DEBUG REMOVE*/
  time_counter = 0;
  step = 0;
  start_ticks = 0;


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE", send_att);
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_REF", send_att_ref);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_REF_QUAT", send_ahrs_ref_quat);
  /* debug messages REMOVE*/
  register_periodic_telemetry(DefaultPeriodic, "CONTROL_TEST", send_control_test);
#endif
}

void stabilization_attitude_enter(void) {

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  stabilization_attitude_ref_enter();

  INT32_QUAT_ZERO(stabilization_att_sum_err_quat);
  INT_EULERS_ZERO(stabilization_att_sum_err);

}

void stabilization_attitude_set_failsafe_setpoint(void) {
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy) {
  // stab_att_sp_euler.psi still used in ref..
  memcpy(&stab_att_sp_euler, rpy, sizeof(struct Int32Eulers));

  quat_from_rpy_cmd_i(&stab_att_sp_quat, &stab_att_sp_euler);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading) {
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))

/* calculate thrust setting COMMAND_THRUST based on desired thrust TODO:INT */
void attitude_tcommand_from_t(int16_t *tcom, int32_t *t){

	float tcom_f;
	float thrust_f;
	thrust_f = FLOAT_OF_BFP(*t,INT32_STAB_ALT_T_FRAC);

	float a;
	float b;
	float c;
	a = c_t_rpm_2_f*c_rpm_cmd_f*c_rpm_cmd_f;
	b = c_t_rpm_2_f*2.*c_rpm_f*c_rpm_cmd_f - c_t_rpm_f*c_rpm_cmd_f;
	c = c_t_rpm_2_f*c_rpm_f*c_rpm_f - c_t_rpm_f*c_rpm_f + c_t_f - thrust_f;

	if (thrust_f <= 0.1171)
		*tcom = 0;
	else {
		tcom_f = (-b + sqrtf(b*b- 4.*a*c))/(2.*a);
	  *tcom = ((int16_t)tcom_f);
	}
}

/* calculate real thrust based on thrust setting COMMAND_THRUST */
void attitude_t_from_tcommand(int32_t *t, int16_t *tcom){

	int32_t rpm; // with INT32_RPM_FRAC, static rpm corresponding with tcom

	rpm = (c_rpm >> (INT32_C_RPM_FRAC - INT32_RPM_FRAC)) + (c_rpm_cmd*((int32_t) *tcom) >> (INT32_C_RPM_CMD_FRAC - INT32_RPM_FRAC));

	*t = ((((int64_t)c_t_rpm_2)*((int64_t)rpm)*((int64_t)rpm)) >> ((INT32_C_T_RPM_2_FRAC + INT32_RPM_FRAC + INT32_RPM_FRAC) - INT32_STAB_ALT_T_FRAC))
			- (((int64_t)c_t_rpm)*((int64_t)rpm) >> ((INT32_C_T_RPM_FRAC + INT32_RPM_FRAC) - INT32_STAB_ALT_T_FRAC))
			+ (c_t << (INT32_STAB_ALT_T_FRAC - INT32_C_T_FRAC));
}

/* calculate thrust difference required for desired yaw torque*/
void attitude_tdiff_from_tau_command(int32_t *tdiff, int32_t *tau_des){

	/*TODO: real relation, should be linear as T and tau are in the same order related to the input (theoretically) */
	int32_t gain;
	gain = 20; // thrust/torque/4
	*tdiff = *tau_des*gain; //INT32_STAB_ALT_T_FRAC

}

/* thrust limiter, tavg +- available tdiff yaw +- pitch and roll tdiff, apply limits*/
static void attitude_limit_t(void){

//	/*DEBUG REMOVE*/
//	rate_thrust_diff.yaw = 0;
//	rate_thrust_diff.pitch = 0;
//	rate_thrust_diff.roll = 0;
//	int16_t tcom = 3000;
//	attitude_t_from_tcommand(&altitude_t_avg,&tcom);

  /*DEBUG REMOVE*/
	if (tau_step && autopilot_mode == AP_MODE_TUNE_NDI && stabilization_override_on){
		rate_thrust_diff.yaw = BFP_OF_REAL(tau_step,INT32_STAB_ALT_T_FRAC);
	}

	/* limit tavg between 0 and max thrust */
	if (altitude_t_avg > getMaxTavg())
		altitude_t_avg = getMaxTavg();
	else if (altitude_t_avg < 0)
		altitude_t_avg = 0;

	/* find available tdiff for yaw control */
	int32_t max_yaw_diff;
	if ((getMaxTavg() - altitude_t_avg) < (altitude_t_avg - 0))
		max_yaw_diff = getMaxTavg() - altitude_t_avg;
	else
		max_yaw_diff = altitude_t_avg - 0;

	/* limit thrust diff for yaw control between max and -max */
	if (rate_thrust_diff.yaw > max_yaw_diff)
		rate_thrust_diff.yaw  = max_yaw_diff;
	else if (rate_thrust_diff.yaw  < -max_yaw_diff)
		rate_thrust_diff.yaw  = -max_yaw_diff;

	/* add tdiff yaw to tavg, 1 CW, 2 CCW, 3 CW, 4 CCW (seen from above)*/
	attitude_thrust.T1 = altitude_t_avg - rate_thrust_diff.yaw;
	attitude_thrust.T2 = altitude_t_avg + rate_thrust_diff.yaw;
	attitude_thrust.T3 = altitude_t_avg - rate_thrust_diff.yaw;
	attitude_thrust.T4 = altitude_t_avg + rate_thrust_diff.yaw;

	/* add tdiff pitch and roll, NW, NE, SE, SW (seen from above) */
	attitude_thrust.T1 = attitude_thrust.T1 + rate_thrust_diff.pitch/4 + rate_thrust_diff.roll/4;
	attitude_thrust.T2 = attitude_thrust.T2 + rate_thrust_diff.pitch/4 - rate_thrust_diff.roll/4;
	attitude_thrust.T3 = attitude_thrust.T3 - rate_thrust_diff.pitch/4 - rate_thrust_diff.roll/4;
	attitude_thrust.T4 = attitude_thrust.T4 - rate_thrust_diff.pitch/4 + rate_thrust_diff.roll/4;

	/* limit between 0 and max thrust */
	int32_t max_t;
	int16_t max_t_com = 9600;

	attitude_t_from_tcommand(&max_t,&max_t_com);

	if (attitude_thrust.T1 > max_t)
		attitude_thrust.T1 = max_t;
	else if (attitude_thrust.T1  < 0)
		attitude_thrust.T1  = 0;

	if (attitude_thrust.T2 > max_t)
		attitude_thrust.T2 = max_t;
	else if (attitude_thrust.T2  < 0)
		attitude_thrust.T2  = 0;

	if (attitude_thrust.T3 > max_t)
		attitude_thrust.T3 = max_t;
	else if (attitude_thrust.T3  < 0)
		attitude_thrust.T3  = 0;

	if (attitude_thrust.T4 > max_t)
		attitude_thrust.T4 = max_t;
	else if (attitude_thrust.T4  < 0)
		attitude_thrust.T4  = 0;

}

/* estimate the voltage level if batt in rest TODO:INT*/
static void attitude_calc_vrest(void){
	int64_t c_cmd_2 = ((0.0000000083)*((int64_t)1<<(INT64_C_BAT_FRAC))); // with INT64_C_BAT_FRAC
	int64_t c_cmd = ((0.00000106)*((int64_t)1<<(INT64_C_BAT_FRAC))); // with INT64_C_BAT_FRAC

	vsupply_rest = ((((c_cmd_2*((int64_t)attitude_t_avg_cmd*(int64_t)attitude_t_avg_cmd)) >> (INT64_C_BAT_FRAC - INT32_VREST_FRAC))
			+ BFP_OF_REAL(electrical.vsupply,INT32_VREST_FRAC)/10) << INT32_VREST_FRAC)
			/(BFP_OF_REAL(1,INT32_VREST_FRAC) - ((c_cmd*((int64_t)attitude_t_avg_cmd)) >> (INT64_C_BAT_FRAC - INT32_VREST_FRAC)));

}

/* calculate maximum thrust */
int32_t getMaxTavg(void){

	int64_t C_T_2 = ((0.0000051485)*((int64_t)1<<(INT64_PCH_C_T_FRAC))); //INT64_PCH_C_T_FRAC
	int64_t C_T = ((0.0005651)*((int64_t)1<<(INT64_PCH_C_T_FRAC))); //INT64_PCH_C_T_FRAC
	int32_t C_T_c = BFP_OF_REAL(0.04949,INT32_PCH_T_FRAC); //INT32_PCH_T_FRAC

	int32_t max_thrust; //with INT32_STAB_ALT_T_FRAC
	int64_t rpm_2; // with INT32_PCH_OMEGA_FRAC
	int64_t rpm; // with INT32_PCH_OMEGA_FRAC
	float rpm_2_f;
	float a = -0.000002981;
	float b;
	float c;
	float rpm_f;

	attitude_calc_vrest();

	b = -0.0176 - 0.00002295*(FLOAT_OF_BFP(vsupply_rest,INT32_VREST_FRAC));
	c = 1.2345 + 0.9958*(FLOAT_OF_BFP(vsupply_rest,INT32_VREST_FRAC));

	if ((b*b - 4.*a*c) > 0)
		rpm_f = (-b - sqrtf(b*b - 4.*a*c))/(2.*a);
	else
		rpm_f = 181;

	if (rpm_f > 665)
		rpm_f = 665;
	else if (rpm_f < 181)
		rpm_f = 181;

	rpm_2_f = rpm_f*rpm_f;

	rpm_2 = BFP_OF_REAL(rpm_2_f,INT32_PCH_OMEGA_FRAC);
	rpm = BFP_OF_REAL(rpm_f,INT32_PCH_OMEGA_FRAC);
	max_thrust = (int32_t)((C_T_2*rpm_2 >> ((INT64_PCH_C_T_FRAC + INT32_PCH_OMEGA_FRAC) - INT32_STAB_ALT_T_FRAC))
			- (C_T*rpm >> ((INT64_PCH_C_T_FRAC + INT32_PCH_OMEGA_FRAC) - INT32_STAB_ALT_T_FRAC)))
			+ (C_T_c >> (INT32_PCH_T_FRAC - INT32_STAB_ALT_T_FRAC));

	return max_thrust;
}

static void attitude_run_ff(struct Int32Rates *rate_sp, struct Int32Rates *ref_rate)
{

  /* Compute feedforward based on reference acceleration */
	rate_sp->p = ref_rate->p;
	rate_sp->q = ref_rate->q;
	rate_sp->r = ref_rate->r;

}


static void attitude_run_fb(struct Int32Rates *rate_sp, struct Int32NDIAttitudeGains *gains, int32_t *alpha, int32_t *beta,
    int32_t *psi_fb, struct Int32Rates *rate_err)
{
  /* Sin and cos of beta for proportional tilt command distribution */
  int32_t cbeta; // with INT32_TRIG_FRAC
  int32_t sbeta; // with INT32_TRIG_FRAC
  PPRZ_ITRIG_COS(cbeta,*beta)
  PPRZ_ITRIG_SIN(sbeta,*beta)

  /*  Proportional feedback tilt angle */
  int32_t tilt_pcmd; // with INT32_RATE_FRAC
  tilt_pcmd = (GAIN_PRESCALER_P * INT_MULT_RSHIFT(gains->p.tilt, *alpha,ATT_GAINS_FRAC)) >> (INT32_ANGLE_FRAC - INT32_RATE_FRAC);

  /*  Derivative feedback angular rates p and q */
  int32_t p_dcmd; // with INT32_RATE_FRAC
  int32_t q_dcmd; // with INT32_RATE_FRAC
  p_dcmd = GAIN_PRESCALER_D * INT_MULT_RSHIFT(gains->d.tilt,rate_err->p,ATT_GAINS_FRAC);
  q_dcmd = GAIN_PRESCALER_D * INT_MULT_RSHIFT(gains->d.tilt,rate_err->q,ATT_GAINS_FRAC);

  /* Distribution over roll and pitch command by using attitude error axis direction beta TODO: possible difference in pitch and roll response*/
  rate_sp->p = INT_MULT_RSHIFT(tilt_pcmd, cbeta, INT32_TRIG_FRAC) + p_dcmd;
  rate_sp->q = INT_MULT_RSHIFT(tilt_pcmd, sbeta, INT32_TRIG_FRAC) + q_dcmd;

  /* Yaw proportional and derivative feedback */
  rate_sp->r =
  ((GAIN_PRESCALER_P * INT_MULT_RSHIFT(gains->p.yaw, *psi_fb,ATT_GAINS_FRAC)) >> (INT32_ANGLE_FRAC - INT32_RATE_FRAC)) +
  GAIN_PRESCALER_D * INT_MULT_RSHIFT(gains->d.yaw,rate_err->r,ATT_GAINS_FRAC);

}

void stabilization_attitude_thrust_run(bool_t motors_on) {

//	/*DEBUG REMOVE*/
//	int16_t cmd;
//	cmd = -9600;
//	float time_f;
//	float cmd_f;
//	float hz;

	if (autopilot_mode == AP_MODE_KILL || electrical.bat_critical){
		thrust_command[0] = 3000;
		thrust_command[1] = 3000;
		thrust_command[2] = 3000;
		thrust_command[3] = 3000;
	}
	else if (motors_on) {

//		/*DEBUG REMOVE - step of 500 every 10 seconds*/
//		if (time_counter == 0)
//			start_ticks = sys_time.nb_tick;
//
//		time_counter = 1;
//
//    ticks = sys_time.nb_tick - start_ticks;
//    time_f = (float)ticks/(float)sys_time.cpu_ticks_per_sec;
//    hz = 50000*time_f;
//    cmd_f = 4800. + 4800.*sinf(hz*time_f*2.*3.14);
//    cmd = (int16_t)cmd_f;
//
//    thrust_command[0] = cmd;
//    thrust_command[1] = cmd;
//    thrust_command[2] = cmd;
//    thrust_command[3] = cmd;


//		/*DEBUG REMOVE - step of 500 every 10 seconds*/
//		if (time_counter == 0)
//			start_time = sys_time.nb_sec;
//
//		time_counter = 1;
//
//    step = (sys_time.nb_sec - start_time)/10;
//
//    cmd = step*500+9000;
//
//    if (step < 20){
//			thrust_command[0] = cmd;
//			thrust_command[1] = cmd;
//			thrust_command[2] = cmd;
//			thrust_command[3] = cmd;
//    }
//    else if (step == 20){
//    	thrust_command[0] = 9600;
//    	thrust_command[1] = 9600;
//    	thrust_command[2] = 9600;
//    	thrust_command[3] = 9600;
//    }
//    else{
//    	thrust_command[0] = -9600;
//    	thrust_command[1] = -9600;
//    	thrust_command[2] = -9600;
//    	thrust_command[3] = -9600;
//    }

//		/*DEBUG REMOVE - step of -500 every 10 seconds*/
//		if (time_counter == 0)
//			start_time = sys_time.nb_sec;
//
//		time_counter = 1;
//
//		step = (sys_time.nb_sec - start_time)/10;
//
//		cmd = step*500;
//
//    if (step == 0){
//    	thrust_command[0] = 9600;
//    	thrust_command[1] = 9600;
//    	thrust_command[2] = 9600;
//    	thrust_command[3] = 9600;
//    }
//
//		if (step < 20){
//			thrust_command[0] = 9500 - cmd;
//			thrust_command[1] = 9500 - cmd;
//			thrust_command[2] = 9500 - cmd;
//			thrust_command[3] = 9500 - cmd;
//		}
//		else if (step == 20){
//			thrust_command[0] = 0;
//			thrust_command[1] = 0;
//			thrust_command[2] = 0;
//			thrust_command[3] = 0;
//		}
//		else{
//			thrust_command[0] = -9600;
//			thrust_command[1] = -9600;
//			thrust_command[2] = -9600;
//			thrust_command[3] = -9600;
//		}

//		/*DEBUG REMOVE - 0 to step to 0, 10 sec per part*/
//		if (time_counter == 0)
//			start_time = sys_time.nb_sec;
//
//		time_counter = 1;
//
//    step = (sys_time.nb_sec - start_time)/10;
//    cmd = (step*1000)/2;
//
//			if (step == 0 || step == 1 || step == 3 || step == 5 ||
//					step == 7 || step == 9 || step == 11 || step == 13 ||
//					step == 15 || step == 17 || step == 19 || step == 21){
//				thrust_command[0] = 0;
//				thrust_command[1] = 0;
//				thrust_command[2] = 0;
//				thrust_command[3] = 0;
//			}
//			else if (step < 20){
//				thrust_command[0] = cmd;
//				thrust_command[1] = cmd;
//				thrust_command[2] = cmd;
//				thrust_command[3] = cmd;
//			}
//			else if (step == 20){
//				thrust_command[0] = 9600;
//				thrust_command[1] = 9600;
//				thrust_command[2] = 9600;
//				thrust_command[3] = 9600;
//			}
//			else{
//				thrust_command[0] = -9600;
//				thrust_command[1] = -9600;
//				thrust_command[2] = -9600;
//				thrust_command[3] = -9600;
//			}
//
//		/*DEBUG REMOVE - 7500 to step to 7500, 10 sec per part*/
//		if (time_counter == 0)
//			start_time = sys_time.nb_sec;
//
//		time_counter = 1;
//
//		step = (sys_time.nb_sec - start_time)/10;
//		cmd = (step*1000)/2;
//
//		if (step == 0 || step == 1 || step == 3 || step == 5 ||
//				step == 7 || step == 9 || step == 11 || step == 13 ||
//				step == 15){
//			thrust_command[0] = 7500;
//			thrust_command[1] = 7500;
//			thrust_command[2] = 7500;
//			thrust_command[3] = 7500;
//		}
//		else if (step < 15){
//			thrust_command[0] = cmd;
//			thrust_command[1] = cmd;
//			thrust_command[2] = cmd;
//			thrust_command[3] = cmd;
//		}
//		else{
//			thrust_command[0] = -9600;
//			thrust_command[1] = -9600;
//			thrust_command[2] = -9600;
//			thrust_command[3] = -9600;
//		}

  	thrust_command[0] = attitude_thrust_command.T1;
  	thrust_command[1] = attitude_thrust_command.T2;
  	thrust_command[2] = attitude_thrust_command.T3;
  	thrust_command[3] = attitude_thrust_command.T4;
  }
  else {
  	thrust_command[0] = -9600;
  	thrust_command[1] = -9600;
  	thrust_command[2] = -9600;
  	thrust_command[3] = -9600;
  }

	attitude_t_avg_cmd = (thrust_command[0] + thrust_command[1] + thrust_command[2] + thrust_command[3])/4;

	stabilization_cmd[COMMAND_THRUST] = attitude_t_avg_cmd;
	stabilization_cmd[COMMAND_PITCH] = 0;
	stabilization_cmd[COMMAND_ROLL] = 0;
	stabilization_cmd[COMMAND_YAW] = 0;

	if (attitude_t_avg_cmd < 0)
			attitude_t_avg_cmd = 0;

}

void stabilization_attitude_run(bool_t enable_integrator) {

//	/*DEBUG REMOVE*/
//	enable_integrator = TRUE;

	/*update pch model with previous thrust setting*/
	stabilization_pch_update();

  /* set altitude setpoints in case of KILL or FAILSAFE */
	if (autopilot_mode == AP_MODE_KILL){
		altitude_z_sp = 0;
		altitude_zd_sp = 0;
	}
	if (autopilot_mode == AP_MODE_FAILSAFE){
		altitude_z_sp = (int64_t)(stateGetPositionNed_i()->z << (INT64_STAB_ALT_X_REF_FRAC - INT32_POS_FRAC));
	  altitude_zd_sp = ((0.5)*(((int64_t)1)<<(36)));
	}

	/* run altitude controller to find quat_sp and altitude_t_avg */
  stabilization_altitude_run(enable_integrator);
  stab_att_ndi_sp_quat = altitude_attitude_sp;

  /*
   * Update reference
   */

	/* calculate PCH correction v_h_att based on previous desired rate and rate ref model rate (which is corrected by PCH)*/
  struct Int64Rates rate_ref_scaled_vh; // INT32_RATE_FRAC
  rate_ref_scaled_vh.p = stab_rate_ref.p >> (RATE_REF_RATE_FRAC - INT32_RATE_FRAC);
  rate_ref_scaled_vh.q = stab_rate_ref.q >> (RATE_REF_RATE_FRAC - INT32_RATE_FRAC);
  rate_ref_scaled_vh.r = stab_rate_ref.r >> (RATE_REF_RATE_FRAC - INT32_RATE_FRAC);

  v_h_att.p = (int32_t)(stab_rate_sp.p - rate_ref_scaled_vh.p);
  v_h_att.q = (int32_t)(stab_rate_sp.q - rate_ref_scaled_vh.q);
  v_h_att.r = (int32_t)(stab_rate_sp.r - rate_ref_scaled_vh.r);

  stabilization_attitude_ref_update();

  /*
  * Compute errors for feedback
  */

  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat* att_quat = stateGetNedToBodyQuat_i();
  INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_ref_quat);

  /* wrap it in the shortest direction       */
  INT32_QUAT_WRAP_SHORTEST(att_err);
  INT32_QUAT_NORMALIZE(att_err);

  /* Split error => roll and pitch separate from yaw */

  /* Find tilt error angle alpha */
  int64_t arg_acos;
  arg_acos = (1<<2*INT32_QUAT_FRAC) - 2*(att_err.qx*att_err.qx + att_err.qy*att_err.qy);

  /* TODO: acos in BFP*/
  int32_t alpha;
  float alpha_f;
  alpha_f = acosf(QUAT1_FLOAT_OF_BFP(QUAT1_FLOAT_OF_BFP(arg_acos)));
  alpha = ANGLE_BFP_OF_REAL(alpha_f);

  /* Find tilt error angle axis direction beta and yaw error angle psi */

  /* TODO: atan2, sin and cos in BFP, current available functions are not precise enough */
  int32_t beta, psi, cpsi_2, spsi_2, salpha_2;
  float salpha_2_f, q3_f, q0_f, rx, ry, cpsi_2_f, spsi_2_f, beta_f;

  q3_f = QUAT1_FLOAT_OF_BFP(att_err.qz);
  q0_f = QUAT1_FLOAT_OF_BFP(att_err.qi);

  salpha_2_f = sinf(alpha_f/2);
  salpha_2 = TRIG_BFP_OF_REAL(salpha_2_f);

  psi_f = 2*atan2f(q3_f,q0_f);
  psi = ANGLE_BFP_OF_REAL(psi_f);

  cpsi_2_f = cosf(psi_f/2);
  cpsi_2 = TRIG_BFP_OF_REAL(cpsi_2_f);
  spsi_2_f = sinf(psi_f/2);
  spsi_2 = TRIG_BFP_OF_REAL(spsi_2_f);

  if (alpha == 0 || salpha_2_f == 0) {
	  beta = 0;
	  beta_f = 0;
  }
  else {

	  rx =
			  TRIG_FLOAT_OF_BFP((INT_MULT_RSHIFT(cpsi_2,att_err.qx,INT32_QUAT_FRAC) -
					  INT_MULT_RSHIFT(spsi_2,att_err.qy,INT32_QUAT_FRAC)))/TRIG_FLOAT_OF_BFP(salpha_2);

	  ry =
			  TRIG_FLOAT_OF_BFP((INT_MULT_RSHIFT(spsi_2,att_err.qx,INT32_QUAT_FRAC) +
					  INT_MULT_RSHIFT(cpsi_2,att_err.qy,INT32_QUAT_FRAC)))/TRIG_FLOAT_OF_BFP(salpha_2);

	  beta_f = atan2f(ry,rx);
	  beta = ANGLE_BFP_OF_REAL(beta_f);
  }

  /*  Desired rate in body frame (splitting yaw and tilt) TODO: check results */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(stab_att_ref_rate.p, (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.q, (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.r, (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC)) };

  struct Int32Rates rate_ref_body;
  int32_t cpsi, spsi;
  cpsi = TRIG_BFP_OF_REAL(cosf(psi_f));
  spsi = TRIG_BFP_OF_REAL(sinf(psi_f));
  rate_ref_body.p = INT_MULT_RSHIFT(rate_ref_scaled.p,cpsi,INT32_TRIG_FRAC) - INT_MULT_RSHIFT(rate_ref_scaled.q,spsi,INT32_TRIG_FRAC);
  rate_ref_body.q = INT_MULT_RSHIFT(rate_ref_scaled.q,cpsi,INT32_TRIG_FRAC) + INT_MULT_RSHIFT(rate_ref_scaled.p,spsi,INT32_TRIG_FRAC);
  rate_ref_body.r = rate_ref_scaled.r;

  /* rate error */
  struct Int32Rates rate_err;
  struct Int32Rates* body_rate = stateGetBodyRates_i();
  RATES_DIFF(rate_err, rate_ref_body, (*body_rate));

  /* compute the feed back command */
  attitude_run_fb(&stab_rate_sp_fb, &attitude_ndi_gains, &alpha, &beta, &psi, &rate_err);

  /*compute feed forward command*/
  struct Int32Rates stab_att_ref_ff_body;
  stab_att_ref_ff_body.p = (int32_t)(((((int64_t)stab_att_ref_ff.p)*((int64_t)cpsi)) >> INT32_TRIG_FRAC) - ((((int64_t)stab_att_ref_ff.q)*((int64_t)spsi)) >> INT32_TRIG_FRAC));
  stab_att_ref_ff_body.q = (int32_t)(((((int64_t)stab_att_ref_ff.q)*((int64_t)cpsi)) >> INT32_TRIG_FRAC) + ((((int64_t)stab_att_ref_ff.p)*((int64_t)spsi)) >> INT32_TRIG_FRAC));//(int32_t)(INT_MULT_RSHIFT((int64_t)stab_att_ref_ff.q,(int64_t)cpsi,INT32_TRIG_FRAC) - INT_MULT_RSHIFT((int64_t)stab_att_ref_ff.p,(int64_t)spsi,INT32_TRIG_FRAC));
  stab_att_ref_ff_body.r = stab_att_ref_ff.r;

  attitude_run_ff(&stab_rate_sp_ff,&stab_att_ref_ff_body);

//  /*DEBUG REMOVE*/
//  stab_rate_sp_fb.p = 0;
//  stab_rate_sp_fb.q = 0;
//  stab_rate_sp_fb.r = 0;

//  /*DEBUG REMOVE*/
//  alt_test1 = FLOAT_OF_BFP(v_h_att.q,INT32_RATE_FRAC);

  stab_rate_sp.p = stab_rate_sp_ff.p + stab_rate_sp_fb.p;
  stab_rate_sp.q = stab_rate_sp_ff.q + stab_rate_sp_fb.q;
  stab_rate_sp.r = stab_rate_sp_ff.r + stab_rate_sp_fb.r;

  /* run inner loop rate control, calculates thrust changes for attitude control rate_thrust */

  /*DEBUG REMOVE*/
  if (att_d_sp && autopilot_mode == AP_MODE_TUNE_NDI && stabilization_override_on){
  	stab_rate_sp.p = RATE_BFP_OF_REAL(phi_d_sp);
  	stab_rate_sp.q = RATE_BFP_OF_REAL(theta_d_sp);
  	stab_rate_sp.r = RATE_BFP_OF_REAL(psi_d_sp);
  }

  stabilization_rate_ndi_run(enable_integrator);

  /* limit thrusts with pitch and roll priority over tavg and tavg priority over yaw */

  attitude_limit_t();

  /* calculate thrust command values */
  attitude_tcommand_from_t(&attitude_thrust_command.T1, &attitude_thrust.T1);
  attitude_tcommand_from_t(&attitude_thrust_command.T2, &attitude_thrust.T2);
  attitude_tcommand_from_t(&attitude_thrust_command.T3, &attitude_thrust.T3);
  attitude_tcommand_from_t(&attitude_thrust_command.T4, &attitude_thrust.T4);

  /* limit command values */
  if (attitude_thrust_command.T1 > 9600) {
  	attitude_thrust_command.T1 = 9600;
  }
  else if (attitude_thrust_command.T1 < 0){
  	attitude_thrust_command.T1 = 0;
  }

  if (attitude_thrust_command.T2 > 9600) {
  	attitude_thrust_command.T2 = 9600;
  }
  else if (attitude_thrust_command.T2 < 0){
  	attitude_thrust_command.T2 = 0;
  }

  if (attitude_thrust_command.T3 > 9600) {
  	attitude_thrust_command.T3 = 9600;
  }
  else if (attitude_thrust_command.T3 < 0){
  	attitude_thrust_command.T3 = 0;
  }

  if (attitude_thrust_command.T4 > 9600) {
  	attitude_thrust_command.T4 = 9600;
  }
  else if (attitude_thrust_command.T4 < 0){
  	attitude_thrust_command.T4 = 0;
  }

  //  /*DEBUG REMOVE test messages*/

//    struct Int32Eulers testje;
//    INT32_EULERS_OF_QUAT(testje,stab_att_ref_quat);
//    test1 = FLOAT_OF_BFP(testje.phi,INT32_ANGLE_FRAC);
//    test2 = FLOAT_OF_BFP(testje.theta,INT32_ANGLE_FRAC);
//    test3 = FLOAT_OF_BFP(testje.psi,INT32_ANGLE_FRAC);
//  test2 = 0;
//  test3 = FLOAT_OF_BFP(stab_att_ref_quat.qy,INT32_QUAT_FRAC);
//  test4 = FLOAT_OF_BFP(stab_att_ref_quat.qz,INT32_QUAT_FRAC);
//  test5 = FLOAT_OF_BFP(stab_att_sp_quat.qi,INT32_QUAT_FRAC);
//  test6 = FLOAT_OF_BFP(stab_att_sp_quat.qx,INT32_QUAT_FRAC);
//  test7 = FLOAT_OF_BFP(stab_att_sp_quat.qy,INT32_QUAT_FRAC);
//  test8 = FLOAT_OF_BFP(stab_att_sp_quat.qz,INT32_QUAT_FRAC);
//  test9 = FLOAT_OF_BFP(stateGetNedToBodyQuat_i()->qi,INT32_QUAT_FRAC);
//  test10 = FLOAT_OF_BFP(stateGetNedToBodyQuat_i()->qx,INT32_QUAT_FRAC);
//  test11 = FLOAT_OF_BFP(stateGetNedToBodyQuat_i()->qy,INT32_QUAT_FRAC);
//  test12 = FLOAT_OF_BFP(stateGetNedToBodyQuat_i()->qz,INT32_QUAT_FRAC);
//  test13 = alt_test1;
//  test14 = alt_test2;
//  test15 = FLOAT_OF_BFP(stateGetPositionNed_i()->z,INT32_POS_FRAC);

  /*DEBUG REMOVE data acquisition*/
  test1 = FLOAT_OF_BFP(stab_att_ref_quat.qi,INT32_QUAT_FRAC);
  test2 = FLOAT_OF_BFP(stab_att_ref_quat.qx,INT32_QUAT_FRAC);
  test3 = FLOAT_OF_BFP(stab_att_ref_quat.qy,INT32_QUAT_FRAC);
  test4 = FLOAT_OF_BFP(stab_att_ref_quat.qz,INT32_QUAT_FRAC);
  test5 = FLOAT_OF_BFP(stab_att_sp_quat.qi,INT32_QUAT_FRAC);
  test6 = FLOAT_OF_BFP(stab_att_sp_quat.qx,INT32_QUAT_FRAC);
  test7 = FLOAT_OF_BFP(stab_att_sp_quat.qy,INT32_QUAT_FRAC);
  test8 = FLOAT_OF_BFP(stab_att_sp_quat.qz,INT32_QUAT_FRAC);
  test9 = FLOAT_OF_BFP(stateGetNedToBodyQuat_i()->qi,INT32_QUAT_FRAC);
  test10 = FLOAT_OF_BFP(stateGetNedToBodyQuat_i()->qx,INT32_QUAT_FRAC);
  test11 = FLOAT_OF_BFP(stateGetNedToBodyQuat_i()->qy,INT32_QUAT_FRAC);
  test12 = FLOAT_OF_BFP(stateGetNedToBodyQuat_i()->qz,INT32_QUAT_FRAC);
  test13 = alt_test1;
  test14 = alt_test2;
  test15 = FLOAT_OF_BFP(stateGetPositionNed_i()->z,INT32_POS_FRAC);
  test16 = thrust_command[0];
  test17 = thrust_command[1];
  test18 = thrust_command[2];
  test19 = thrust_command[3];

//  /*DEBUG REMOVE test*/
//  test1 = 0;
//  test2 = alt_test2;
//  test3 = alt_test1;
//  test4 = 0;
//  test5 = 0;
//  test6 = 0;

//  int16_t tcomtest;
//  int32_t ttest;
//  ttest = BFP_OF_REAL(0.11,INT32_STAB_ALT_T_FRAC);
//  attitude_tcommand_from_t(&tcomtest,&ttest);
//
//	/*DEBUG REMOVE*/
//  alt_test1 = tcomtest;//FLOAT_OF_BFP(ttest,INT32_STAB_ALT_T_FRAC);
//
//  int32_t tdifftest;
//  int32_t tautest;
//  tautest = BFP_OF_REAL(0.4,INT32_STAB_ALT_T_FRAC);
//  attitude_tdiff_from_tau_command(&tdifftest,&tautest);
//
//	/*DEBUG REMOVE*/
//  alt_test1 = FLOAT_OF_BFP(tdifftest,INT32_STAB_ALT_T_FRAC);
//
//  /*DEBUG REMOVE test*/
//  test1 = 0;
//  test2 = 0;
//  test3 = alt_test1;
//  test4 = 0;
//  test5 = 0;
//  test6 = 0;

//  /*DEBUG REMOVE test calculate right values out of test1!!!!!!!!!!!*/
//  test1 = ((attitude_thrust_command.T2 + attitude_thrust_command.T4) - (attitude_thrust_command.T1 + attitude_thrust_command.T3))/2.;
//  test2 = stateGetBodyRates_f()->r;
//  test3 = 0;
//  test4 = 0;
//  test5 = 0;
//  test6 = 0;

//
//    /*DEBUG REMOVE test*/
//    test1 = electrical.vsupply;
//    test2 = thrust_command[0];
//    test3 = thrust_command[1];
//    test4 = thrust_command[2];
//    test5 = thrust_command[3];
//    test6 = 0;

//  /*DEBUG REMOVE test*/
//  test1 = FLOAT_OF_BFP(altitude_t_avg,INT32_STAB_ALT_T_FRAC);
//  test2 = FLOAT_OF_BFP(rate_thrust_diff.pitch,INT32_STAB_ALT_T_FRAC);
//  test3 = rate_test;
//  test4 = 0;
//  test5 = 0;
//  test6 = 0;

//  /*DEBUG REMOVE yaw rate tuning*/
//  test1 = FLOAT_OF_BFP(rate_thrust_diff.yaw,INT32_STAB_ALT_T_FRAC);
//  test2 = ((float)(stab_rate_sp.r)/((int32_t)1<<(INT32_RATE_FRAC)));
//  test3 = FLOAT_OF_BFP(stateGetBodyRates_i()->r,INT32_RATE_FRAC);
//  test4 = ((float)(stab_rate_ref.r)/((int64_t)1<<(30)));
//  test5 = rate_test;
//  test6 = FLOAT_OF_BFP(altitude_t_avg,INT32_STAB_ALT_T_FRAC);

//  /*DEBUG REMOVE p rate tuning*/
//  test1 = FLOAT_OF_BFP(rate_thrust_diff.roll,INT32_STAB_ALT_T_FRAC);
//  test2 = ((float)(stab_rate_sp.p)/((int32_t)1<<(INT32_RATE_FRAC)));
//  test3 = FLOAT_OF_BFP(stateGetBodyRates_i()->p,INT32_RATE_FRAC);
//  test4 = ((float)(stab_rate_ref.p)/((int64_t)1<<(30)));
//  test5 = RATE_FLOAT_OF_BFP(stab_rate_sp.p);
//  test6 = rate_test;

//  /*DEBUG REMOVE tilt theta and q rate tuning*/
//  struct Int32Eulers testje;
//  INT32_EULERS_OF_QUAT(testje,stab_att_ref_quat);
//  test1 = ((float)(stab_rate_sp.q)/((int32_t)1<<(INT32_RATE_FRAC)));
//  test2 = FLOAT_OF_BFP(stateGetBodyRates_i()->q,INT32_RATE_FRAC);
//  test3 = ((float)(stab_rate_ref.q)/((int64_t)1<<(30)));
//  struct Int32Eulers testje2;
//  INT32_EULERS_OF_QUAT(testje2,stab_att_sp_quat);
//  test4 = ANGLE_FLOAT_OF_BFP(testje2.theta);
//  test5 = ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->theta);
//  test6 = ANGLE_FLOAT_OF_BFP(testje.theta);
//  test7 = alt_test5;

//  /*DEBUG REMOVE pch tilt test*/
//  test1 = FLOAT_OF_BFP(rate_thrust_diff.roll,INT32_STAB_ALT_T_FRAC);
//  test2 = FLOAT_OF_BFP(rate_thrust_diff.pitch,INT32_STAB_ALT_T_FRAC);
//  test3 = FLOAT_OF_BFP(virtual_input.q,INT32_RATE_FRAC);
//  test4 = FLOAT_OF_BFP(pch_ang_accel.q,INT32_RATE_FRAC);
//  test5 = FLOAT_OF_BFP(pch_ang_accel.r,INT32_RATE_FRAC);
//  test6 = rate_test;

//  /*DEBUG REMOVE pch alt test*/
//  test1 = 0;
//  test2 = 0;
//  test3 = FLOAT_OF_BFP(pch_trans_accel.z,INT32_PCH_F_FRAC);
//  test4 = FLOAT_OF_BFP(pch_ang_accel.q,INT32_RATE_FRAC);
//  test5 = FLOAT_OF_BFP(pch_ang_accel.r,INT32_RATE_FRAC);
//  test6 = alt_test;

//  /*DEBUG REMOVE tilt tuning*/
//  struct Int32Eulers testje;
//  INT32_EULERS_OF_QUAT(testje,stab_att_ref_quat);
//  test1 = FLOAT_OF_BFP(rate_thrust_diff.pitch,INT32_STAB_ALT_T_FRAC);
//  test2 = (theta_sp);
//  test3 = ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->theta);
//  test4 = ANGLE_FLOAT_OF_BFP(testje.theta);
//  test5 = psi_f;
//  test6 = 0;

//  /*DEBUG REMOVE yaw tuning*/
//  struct Int32Eulers testje;
//  INT32_EULERS_OF_QUAT(testje,stab_att_ref_quat);
//  test1 = FLOAT_OF_BFP(rate_thrust_diff.yaw,INT32_STAB_ALT_T_FRAC);
//  test2 = (psi_sp);
//  test3 = ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->psi);
//  test4 = ANGLE_FLOAT_OF_BFP(testje.psi);
//  test5 = psi_f;
//  test6 = 0;

//  int16_t testje = 2260;
//  int32_t test;// = BFP_OF_REAL(0.382,INT32_STAB_ALT_T_FRAC);
//  attitude_t_from_tcommand(&test,&testje);
//  //attitude_tcommand_from_t(&testje,&test);
//
//  /*DEBUG REMOVE*/
//  	alt_test = ((float)(test)/((int32_t)1<<(INT32_STAB_ALT_T_FRAC)));//testje;//
//  /*DEBUG REMOVE*/
//  alt_test = ((float)(stateGetSpeedNed_i()->z)/((int32_t)1<<(INT32_SPEED_FRAC)));
//  /*DEBUG REMOVE test*/
//  test1 = alt_test;
//  test2 = ((float)(stateGetPositionNed_i()->z)/((int32_t)1<<(INT32_POS_FRAC)));
//  test3 = 0;
//  test4 = 0;
//  test5 = 0;
//  test6 = 0;

//  /*DEBUG REMOVE sp's*/
//  struct Int32Eulers testje;
//  INT32_EULERS_OF_QUAT(testje,stab_att_sp_quat);
//  test1 = FLOAT_OF_BFP(testje.phi,INT32_ANGLE_FRAC);
////  test2 = FLOAT_OF_BFP(stab_att_sp_quat.qy,INT32_QUAT_FRAC);
////  test3 = FLOAT_OF_BFP(stab_att_sp_quat.qz,INT32_QUAT_FRAC);
////  test4 = FLOAT_OF_BFP(altitude_attitude_sp.qx,INT32_QUAT_FRAC);
////  test5 = FLOAT_OF_BFP(altitude_attitude_sp.qy,INT32_QUAT_FRAC);
////  test6 = FLOAT_OF_BFP(altitude_attitude_sp.qz,INT32_QUAT_FRAC);

//  /*DEBUG REMOVE altitude tuning */
//  test1 = FLOAT_OF_BFP(stateGetPositionNed_i()->z,INT32_POS_FRAC);
//	test2 = alt_test1;
//  test3 = alt_test2;
//  test4 = 0;
//  test5 = 0;
//  test6 = 0;
//
//  /*DEBUG REMOVE switch tuning*/
//  struct Int32Eulers testje;
//  INT32_EULERS_OF_QUAT(testje,stab_att_ref_quat);
//  test1 = FLOAT_OF_BFP(stateGetPositionNed_i()->z,INT32_POS_FRAC);
//  test2 = alt_test1;
//  test3 = alt_test2;
//  struct Int32Eulers testje2;
//  INT32_EULERS_OF_QUAT(testje2,stab_att_sp_quat);
//  test4 = ANGLE_FLOAT_OF_BFP(testje2.theta);
//  test5 = ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->theta);
//  test6 = ANGLE_FLOAT_OF_BFP(testje.theta);
//  test7 = alt_test5;

//
//  /*DEBUG REMOVE z and zd tuning */
//  test1 = FLOAT_OF_BFP(stateGetPositionNed_i()->z,INT32_POS_FRAC);
//	test2 = alt_test1;
//  test3 = alt_test2;
//  test4 = FLOAT_OF_BFP(stateGetSpeedNed_i()->z,INT32_SPEED_FRAC);
//  test5 = alt_test3;
//  test6 = alt_test4;

//  /*DEBUG REMOVE vz tuning */
//  test1 = FLOAT_OF_BFP(stateGetSpeedNed_i()->z,INT32_SPEED_FRAC);
//	test2 = alt_test1;
//  test3 = alt_test2;
//  test4 = 0;
//  test5 = 0;
//  test6 = 0;
//
//  /*DEBUG REMOVE tilt tuning*/
//  struct Int32Eulers testje;
//  INT32_EULERS_OF_QUAT(testje,stab_att_ref_quat);
//  test7 = FLOAT_OF_BFP(stab_att_ndi_sp_quat.qy,INT32_QUAT_FRAC);
//  test8 = ANGLE_FLOAT_OF_BFP(stateGetNedToBodyEulers_i()->theta);
//  test9 = ANGLE_FLOAT_OF_BFP(testje.theta);

//  /*DEBUG REMOVE z tuning */
//  test1 = FLOAT_OF_BFP(stateGetPositionNed_i()->z,INT32_POS_FRAC);
//	test2 = alt_test;
//  test3 = 0;
//  test4 = 0;
//  test5 = 0;
//  test6 = 0;

//  /*DEBUG REMOVE sp's*/
//  test1 = attitude_thrust.T1;
//  test2 = attitude_thrust.T2;
//  test3 = attitude_thrust.T3;
//  test4 = attitude_thrust.T4;
//  test5 = 0;
//  test6 = 0;
}

void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn) {
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);

	/*DEBUG REMOVE tune altitude*/
	if ((alt_sp || alt_d_sp) && autopilot_mode == AP_MODE_TUNE_NDI && stabilization_override_on){

		stab_att_sp_quat.qi = QUAT1_BFP_OF_REAL(cosf(phi_sp/2)*cosf(theta_sp/2)*cosf(psi_sp/2) + sinf(phi_sp/2)*sinf(theta_sp/2)*sinf(psi_sp/2));
		stab_att_sp_quat.qx = QUAT1_BFP_OF_REAL(-cosf(phi_sp/2)*sinf(theta_sp/2)*sinf(psi_sp/2) + cosf(theta_sp/2)*cosf(psi_sp/2)*sinf(phi_sp/2));
		stab_att_sp_quat.qy = QUAT1_BFP_OF_REAL(cosf(phi_sp/2)*cosf(psi_sp/2)*sinf(theta_sp/2) + sinf(phi_sp/2)*cosf(theta_sp/2)*sinf(psi_sp/2));
		stab_att_sp_quat.qz = QUAT1_BFP_OF_REAL(cosf(phi_sp/2)*cosf(theta_sp/2)*sinf(psi_sp/2) - sinf(phi_sp/2)*cosf(psi_sp/2)*sinf(theta_sp/2));

	}

  /*DEBUG REMOVE tune attitude*/
  if (att_sp && autopilot_mode == AP_MODE_TUNE_NDI && stabilization_override_on){

//  	/* DEBUG REMOVE going through switch */
//  	if (time_counter == 0)
//  		start_time = sys_time.nb_sec;

//  	uint32_t time;
//  	time_counter = 1;
//  	time = sys_time.nb_sec - start_time;
//
//  	theta_sp = 0.52;
//
//  	if (time > 3.0)
//  		theta_sp = 0.62;

//  	/* DEBUG REMOVE step up down */
//  	if (time_counter == 0)
//  		start_time = sys_time.nb_sec;
//
//  	time_counter = 1;
//
//  	if (sys_time.nb_sec - start_time <= 4)
//  		theta_sp = 0.52;
//  	else
//  		theta_sp = -0.52;

  	stab_att_sp_quat.qi = QUAT1_BFP_OF_REAL(cosf(phi_sp/2.)*cosf(theta_sp/2.)*cosf(psi_sp/2.) + sinf(phi_sp/2.)*sinf(theta_sp/2.)*sinf(psi_sp/2.));
  	stab_att_sp_quat.qx = QUAT1_BFP_OF_REAL(-cosf(phi_sp/2.)*sinf(theta_sp/2.)*sinf(psi_sp/2.) + cosf(theta_sp/2.)*cosf(psi_sp/2.)*sinf(phi_sp/2.));
  	stab_att_sp_quat.qy = QUAT1_BFP_OF_REAL(cosf(phi_sp/2.)*cosf(psi_sp/2.)*sinf(theta_sp/2.) + sinf(phi_sp/2.)*cosf(theta_sp/2.)*sinf(psi_sp/2.));
  	stab_att_sp_quat.qz = QUAT1_BFP_OF_REAL(cosf(phi_sp/2.)*cosf(theta_sp/2.)*sinf(psi_sp/2.) - sinf(phi_sp/2.)*cosf(psi_sp/2.)*sinf(theta_sp/2.));
  }
  else {
//  	time_counter = 0;
//  	theta_sp = 0;
  }

}
