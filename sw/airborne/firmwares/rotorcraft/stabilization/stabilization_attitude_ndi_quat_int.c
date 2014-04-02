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

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include "generated/airframe.h"

#include "std.h"
#include <stdlib.h>

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

int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

#define IERROR_SCALE 1024
#define GAIN_PRESCALER_FF 48
#define GAIN_PRESCALER_P 48
#define GAIN_PRESCALER_D 48
#define GAIN_PRESCALER_I 48

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
                                      &test);
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
  stabilization_rate_ndi_init();

  INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
  INT_EULERS_ZERO( stabilization_att_sum_err );

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

//static void attitude_run_ff(int32_t ff_commands[], struct Int32AttitudeGains *gains, struct Int32Rates *ref_accel)
//{
//  /* Compute feedforward based on reference acceleration */
//
//  ff_commands[COMMAND_ROLL]  = GAIN_PRESCALER_FF * gains->dd.x * RATE_FLOAT_OF_BFP(ref_accel->p) / (1 << 7);
//  ff_commands[COMMAND_PITCH] = GAIN_PRESCALER_FF * gains->dd.y * RATE_FLOAT_OF_BFP(ref_accel->q) / (1 << 7);
//  ff_commands[COMMAND_YAW]   = GAIN_PRESCALER_FF * gains->dd.z * RATE_FLOAT_OF_BFP(ref_accel->r) / (1 << 7);
//}
//

static void attitude_run_fb(int32_t fb_commands[], struct Int32NDIAttitudeGains *gains, int32_t *alpha, int32_t *beta,
    int32_t *psi, struct Int32Rates *rate_err)
{
  /* Sin and cos of beta for proportional tilt command distribution */
  int32_t cbeta;
  int32_t sbeta;
  PPRZ_ITRIG_COS(cbeta,*beta)
  PPRZ_ITRIG_SIN(sbeta,*beta)

  /*  Proportional feedback tilt angle */
  int32_t tilt_pcmd;
  tilt_pcmd = GAIN_PRESCALER_P * gains->p.tilt  * ANGLE_FLOAT_OF_BFP(*alpha) / 4;

  /*  Derivative feedback angular rates p and q */
  int32_t p_dcmd;
  int32_t q_dcmd;
  p_dcmd = GAIN_PRESCALER_D * gains->d.tilt  * RATE_FLOAT_OF_BFP(rate_err->p) / 16;
  q_dcmd = GAIN_PRESCALER_D * gains->d.tilt  * RATE_FLOAT_OF_BFP(rate_err->q) / 16;

  /* Distribution over roll and pitch command by using attitude error axis direction beta TODO: possible difference in pitch and roll response*/
  fb_commands[COMMAND_ROLL] = tilt_pcmd * TRIG_FLOAT_OF_BFP(cbeta) + p_dcmd;
  fb_commands[COMMAND_PITCH] = tilt_pcmd * TRIG_FLOAT_OF_BFP(sbeta) + q_dcmd;

  /* Yaw proportional and derivative feedback */
  fb_commands[COMMAND_YAW] =
  GAIN_PRESCALER_P * gains->p.yaw  * ANGLE_FLOAT_OF_BFP(*psi) / 4 +
  GAIN_PRESCALER_D * gains->d.yaw  * RATE_FLOAT_OF_BFP(rate_err->r)  / 16;

}

void stabilization_attitude_run(bool_t enable_integrator) {

  /*
   * Update reference
   */
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
  int32_t arg_acos;
  arg_acos = (1<<2*INT32_QUAT_FRAC) - 2*(att_err.qx*att_err.qx + att_err.qy*att_err.qy);

  /* TODO: acos in BFP, currently not precise enough */
  int32_t alpha;
  float alpha_f;
  alpha_f = acosf(QUAT1_FLOAT_OF_BFP(QUAT1_FLOAT_OF_BFP(arg_acos)));
  alpha = ANGLE_BFP_OF_REAL(alpha_f);

  /* Find tilt error angle axis direction beta and yaw error angle psi */

  /* TODO: atan2, sin and cos in BFP, current available functions are not precise enough */
  int32_t beta, psi, cpsi_2, spsi_2, salpha_2;
  float salpha_2_f, q3_f, q0_f, psi_f, rx, ry, psi_2_f, cpsi_2_f, spsi_2_f, beta_f;

  salpha_2_f = sinf(alpha_f/2);
  salpha_2 = TRIG_BFP_OF_REAL(salpha_2_f);

  if (alpha == 0 || salpha_2_f == 0) {
	  beta = 0;
	  beta_f = 0;
  }
  else {
	  q3_f = QUAT1_FLOAT_OF_BFP(att_err.qz);
	  q0_f = QUAT1_FLOAT_OF_BFP(att_err.qi);

	  psi_f = atan2f(q3_f,q0_f);
	  psi = ANGLE_BFP_OF_REAL(psi_f);

	  cpsi_2_f = cosf(psi_f/2);
	  cpsi_2 = TRIG_BFP_OF_REAL(cpsi_2_f);
	  spsi_2_f = sinf(psi_f/2);
	  spsi_2 = TRIG_BFP_OF_REAL(spsi_2_f);

	  rx =
			  TRIG_FLOAT_OF_BFP((INT_MULT_RSHIFT(cpsi_2,att_err.qx,INT32_QUAT_FRAC) -
					  INT_MULT_RSHIFT(spsi_2,att_err.qy,INT32_QUAT_FRAC)))/TRIG_FLOAT_OF_BFP(salpha_2);

	  ry =
			  TRIG_FLOAT_OF_BFP((INT_MULT_RSHIFT(spsi_2,att_err.qx,INT32_QUAT_FRAC) +
					  INT_MULT_RSHIFT(cpsi_2,att_err.qy,INT32_QUAT_FRAC)))/TRIG_FLOAT_OF_BFP(salpha_2);

	  beta_f = atan2f(ry,rx);
	  beta = ANGLE_BFP_OF_REAL(beta_f);
  }

  /*  Desired rate in body frame (splitting yaw and tilt) */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC)) };

  struct Int32Rates rate_ref_body;
  int32_t cpsi, spsi;
  cpsi = TRIG_BFP_OF_REAL(cosf(psi_f));
  spsi = TRIG_BFP_OF_REAL(sinf(psi_f));
  rate_ref_body.p = INT_MULT_RSHIFT(rate_ref_scaled.p,cpsi,INT32_TRIG_FRAC) + INT_MULT_RSHIFT(rate_ref_scaled.q,spsi,INT32_TRIG_FRAC);
  rate_ref_body.q = INT_MULT_RSHIFT(rate_ref_scaled.q,cpsi,INT32_TRIG_FRAC) - INT_MULT_RSHIFT(rate_ref_scaled.p,spsi,INT32_TRIG_FRAC);
  rate_ref_body.r = rate_ref_scaled.r;

  /* rate error */
  struct Int32Rates rate_err;
  struct Int32Rates* body_rate = stateGetBodyRates_i();
  RATES_DIFF(rate_err, rate_ref_body, (*body_rate));

//  /* integrated error */
//  if (enable_integrator) {
//    struct Int32Quat new_sum_err, scaled_att_err;
//    /* update accumulator */
//    scaled_att_err.qi = att_err.qi;
//    scaled_att_err.qx = att_err.qx / IERROR_SCALE;
//    scaled_att_err.qy = att_err.qy / IERROR_SCALE;
//    scaled_att_err.qz = att_err.qz / IERROR_SCALE;
//    INT32_QUAT_COMP(new_sum_err, stabilization_att_sum_err_quat, scaled_att_err);
//    INT32_QUAT_NORMALIZE(new_sum_err);
//    QUAT_COPY(stabilization_att_sum_err_quat, new_sum_err);
//    INT32_EULERS_OF_QUAT(stabilization_att_sum_err, stabilization_att_sum_err_quat);
//  } else {
//    /* reset accumulator */
//    INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
//    INT_EULERS_ZERO( stabilization_att_sum_err );
//  }

//  /* compute the feed forward command */
//  attitude_run_ff(stabilization_att_ff_cmd, &stabilization_gains, &stab_att_ref_accel);

  /* compute the feed back command */
  attitude_run_fb(stabilization_att_fb_cmd, &attitude_ndi_gains, &alpha, &beta, &psi, &rate_err);

  /* sum feedforward and feedback */
  stabilization_cmd[COMMAND_ROLL] = stabilization_att_fb_cmd[COMMAND_ROLL];// + stabilization_att_ff_cmd[COMMAND_ROLL];
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_fb_cmd[COMMAND_PITCH];// + stabilization_att_ff_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_fb_cmd[COMMAND_YAW];// + stabilization_att_ff_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);

  // TEST REMOVE
  stab_rate_sp.p = stabilization_cmd[COMMAND_ROLL] << 10;
  stab_rate_sp.q = stabilization_cmd[COMMAND_PITCH] << 10;
  stab_rate_sp.r = stabilization_cmd[COMMAND_YAW] << 10;

  stabilization_rate_ndi_run(enable_integrator);

}

void stabilization_attitude_read_rc(bool_t in_flight) {
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}
