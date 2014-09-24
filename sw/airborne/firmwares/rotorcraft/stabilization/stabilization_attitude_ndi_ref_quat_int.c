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

/**
 * @file firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.c
 *
 * Rotorcraft attitude reference generation.
 * (quaternion int version)
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ndi_quat_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ndi_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_saturate.h"

/*DEBUG REMOVE*/
#include "mcu_periph/sys_time.h"
int32_t counter = 0;
uint32_t prev_ticks = 0;

#define REF_ACCEL_MAX_P BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_PDOT, INT64_ATT_REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_Q BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_QDOT, INT64_ATT_REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_R BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_RDOT, INT64_ATT_REF_ACCEL_FRAC)

#define REF_RATE_MAX_P BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_P, INT64_ATT_REF_RATE_FRAC)
#define REF_RATE_MAX_Q BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_Q, INT64_ATT_REF_RATE_FRAC)
#define REF_RATE_MAX_R BFP_OF_REAL(STABILIZATION_ATTITUDE_REF_MAX_R, INT64_ATT_REF_RATE_FRAC)

//#define OMEGA_P   STABILIZATION_ATTITUDE_REF_OMEGA_P
//#define ZETA_P    STABILIZATION_ATTITUDE_REF_ZETA_P
#define ZETA_OMEGA_P_RES 1
//#define ZETA_OMEGA_P BFP_OF_REAL((ZETA_P*OMEGA_P), ZETA_OMEGA_P_RES)
#define OMEGA_2_P_RES 1
//#define OMEGA_2_P    BFP_OF_REAL((OMEGA_P*OMEGA_P), OMEGA_2_P_RES)
//
//#define OMEGA_Q   STABILIZATION_ATTITUDE_REF_OMEGA_Q
//#define ZETA_Q    STABILIZATION_ATTITUDE_REF_ZETA_Q
#define ZETA_OMEGA_Q_RES 1
//#define ZETA_OMEGA_Q BFP_OF_REAL((ZETA_Q*OMEGA_Q), ZETA_OMEGA_Q_RES)
#define OMEGA_2_Q_RES 1
//#define OMEGA_2_Q    BFP_OF_REAL((OMEGA_Q*OMEGA_Q), OMEGA_2_Q_RES)
//
//#define OMEGA_R   STABILIZATION_ATTITUDE_REF_OMEGA_R
//#define ZETA_R    STABILIZATION_ATTITUDE_REF_ZETA_R
#define ZETA_OMEGA_R_RES 1
//#define ZETA_OMEGA_R BFP_OF_REAL((ZETA_R*OMEGA_R), ZETA_OMEGA_R_RES)
#define OMEGA_2_R_RES 1
//#define OMEGA_2_R    BFP_OF_REAL((OMEGA_R*OMEGA_R), OMEGA_2_R_RES)

struct Int32Eulers stab_att_sp_euler;
struct Int32Quat   stab_att_ndi_sp_quat;
struct Int32Eulers stab_att_ref_euler;
struct Int32Quat   stab_att_ref_quat;
struct Int64Rates  stab_att_ref_rate;
struct Int64Rates  stab_att_ref_accel;
struct Int32Rates  stab_att_ref_ff; // with INT32_RATE_FRAC

struct FloatRefModelNDI stab_att_ref_model_ndi = {
  {STABILIZATION_ATTITUDE_REF_OMEGA_P, STABILIZATION_ATTITUDE_REF_OMEGA_Q, STABILIZATION_ATTITUDE_REF_OMEGA_R},
  {STABILIZATION_ATTITUDE_REF_ZETA_P, STABILIZATION_ATTITUDE_REF_ZETA_Q, STABILIZATION_ATTITUDE_REF_ZETA_R}
};

static inline void reset_psi_ref_from_body(void) {
  //sp has been set from body using stabilization_attitude_get_yaw_i, use that value
  stab_att_ref_euler.psi = stab_att_sp_euler.psi << (REF_ANGLE_FRAC - INT32_ANGLE_FRAC);
  stab_att_ref_rate.r = 0;
  stab_att_ref_accel.r = 0;
}

void stabilization_attitude_ref_init(void) {

  INT_EULERS_ZERO(stab_att_sp_euler);
  INT32_QUAT_ZERO(stab_att_ndi_sp_quat);
  INT_EULERS_ZERO(stab_att_ref_euler);
  INT32_QUAT_ZERO(stab_att_ref_quat);
  INT_RATES_ZERO(stab_att_ref_rate);
  INT_RATES_ZERO(stab_att_ref_accel);

  counter = 0;

  /*
  for (int i = 0; i < STABILIZATION_ATTITUDE_GAIN_NB; i++) {
    RATES_ASSIGN(stab_att_ref_model[i].omega, omega_p[i], omega_q[i], omega_r[i]);
    RATES_ASSIGN(stab_att_ref_model[i].zeta, zeta_p[i], zeta_q[i], zeta_r[i]);
  }
  */

}

void stabilization_attitude_ref_enter(void)
{
  reset_psi_ref_from_body();

  /* convert reference attitude with REF_ANGLE_FRAC to eulers with normal INT32_ANGLE_FRAC */
  struct Int32Eulers ref_eul;
  INT32_EULERS_RSHIFT(ref_eul, stab_att_ref_euler, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));
  INT32_QUAT_OF_EULERS(stab_att_ref_quat, ref_eul);
  INT32_QUAT_WRAP_SHORTEST(stab_att_ref_quat);

  /* set reference rate and acceleration to zero */
  memset(&stab_att_ref_accel, 0, sizeof(struct Int32Rates));
  memset(&stab_att_ref_rate, 0, sizeof(struct Int32Rates));
}

/*
 * Reference
 */
#define DT_UPDATE (1./PERIODIC_FREQUENCY)
// CAUTION! Periodic frequency is assumed to be 512 Hz
// which is equal to >> 9
#define F_UPDATE_RES 9

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))


void stabilization_attitude_ref_update(void) {

//  /* integrate reference attitude            */
//  const struct Int32Rates rate_ref_scaled = {
//    OFFSET_AND_ROUND(stab_att_ref_rate.p, (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC)),
//    OFFSET_AND_ROUND(stab_att_ref_rate.q, (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC)),
//    OFFSET_AND_ROUND(stab_att_ref_rate.r, (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC)) };
//  struct Int32Quat qdot;
//  INT32_QUAT_DERIVATIVE(qdot, rate_ref_scaled, stab_att_ref_quat);
//  //QUAT_SMUL(qdot, qdot, DT_UPDATE);
//  qdot.qi = qdot.qi >> F_UPDATE_RES;
//  qdot.qx = qdot.qx >> F_UPDATE_RES;
//  qdot.qy = qdot.qy >> F_UPDATE_RES;
//  qdot.qz = qdot.qz >> F_UPDATE_RES;
//  QUAT_ADD(stab_att_ref_quat, qdot);
//  INT32_QUAT_NORMALIZE(stab_att_ref_quat);
//
//  /* integrate reference rotational speeds
//   * delta rate = ref_accel * dt
//   * ref_rate = old_ref_rate + delta_rate
//   */
//  const struct Int32Rates delta_rate = {
//         stab_att_ref_accel.p >> ( F_UPDATE_RES + INT64_ATT_REF_ACCEL_FRAC - INT64_ATT_REF_RATE_FRAC),
//         stab_att_ref_accel.q >> ( F_UPDATE_RES + INT64_ATT_REF_ACCEL_FRAC - INT64_ATT_REF_RATE_FRAC),
//         stab_att_ref_accel.r >> ( F_UPDATE_RES + INT64_ATT_REF_ACCEL_FRAC - INT64_ATT_REF_RATE_FRAC)};
//  RATES_ADD(stab_att_ref_rate, delta_rate);
//
//  /* compute reference angular accelerations */
//  struct Int32Quat err;
//  /* compute reference attitude error        */
//  INT32_QUAT_INV_COMP(err, stab_att_ndi_sp_quat, stab_att_ref_quat);
//  /* wrap it in the shortest direction       */
//  INT32_QUAT_WRAP_SHORTEST(err);
//  /* propagate the 2nd order linear model    */
//
//  const struct Int32Rates accel_rate = {
//    ((int32_t)(-2.*ZETA_OMEGA_P) * (stab_att_ref_rate.p >> (INT64_ATT_REF_RATE_FRAC - INT64_ATT_REF_ACCEL_FRAC))) >> (ZETA_OMEGA_P_RES),
//    ((int32_t)(-2.*ZETA_OMEGA_Q) * (stab_att_ref_rate.q >> (INT64_ATT_REF_RATE_FRAC - INT64_ATT_REF_ACCEL_FRAC))) >> (ZETA_OMEGA_Q_RES),
//    ((int32_t)(-2.*ZETA_OMEGA_R) * (stab_att_ref_rate.r >> (INT64_ATT_REF_RATE_FRAC - INT64_ATT_REF_ACCEL_FRAC))) >> (ZETA_OMEGA_R_RES) };
//
//  const struct Int32Rates accel_angle = {
//    ((int32_t)(-OMEGA_2_P) * (err.qx >> (INT32_QUAT_FRAC - INT64_ATT_REF_ACCEL_FRAC))) >> (OMEGA_2_P_RES),
//    ((int32_t)(-OMEGA_2_Q) * (err.qy >> (INT32_QUAT_FRAC - INT64_ATT_REF_ACCEL_FRAC))) >> (OMEGA_2_Q_RES),
//    ((int32_t)(-OMEGA_2_R) * (err.qz >> (INT32_QUAT_FRAC - INT64_ATT_REF_ACCEL_FRAC))) >> (OMEGA_2_R_RES) };
//
//  RATES_SUM(stab_att_ref_accel, accel_rate, accel_angle);
//
//  /* saturate acceleration */
//  const struct Int32Rates MIN_ACCEL = { -REF_ACCEL_MAX_P, -REF_ACCEL_MAX_Q, -REF_ACCEL_MAX_R };
//  const struct Int32Rates MAX_ACCEL = {  REF_ACCEL_MAX_P,  REF_ACCEL_MAX_Q,  REF_ACCEL_MAX_R };
//  RATES_BOUND_BOX(stab_att_ref_accel, MIN_ACCEL, MAX_ACCEL);
//
//  /* saturate angular speed and trim accel accordingly */
//  SATURATE_SPEED_TRIM_ACCEL();

	int64_t zeta_omega_p;
	int64_t zeta_omega_q;
	int64_t zeta_omega_r;
	int64_t omega_2_p;
	int64_t omega_2_q;
	int64_t omega_2_r;

	zeta_omega_p = BFP_OF_REAL(stab_att_ref_model_ndi.omega.p*stab_att_ref_model_ndi.zeta.p,ZETA_OMEGA_P_RES);
	zeta_omega_q = BFP_OF_REAL(stab_att_ref_model_ndi.omega.q*stab_att_ref_model_ndi.zeta.q,ZETA_OMEGA_Q_RES);
	zeta_omega_r = BFP_OF_REAL(stab_att_ref_model_ndi.omega.r*stab_att_ref_model_ndi.zeta.r,ZETA_OMEGA_R_RES);

	omega_2_p = BFP_OF_REAL(stab_att_ref_model_ndi.omega.p*stab_att_ref_model_ndi.omega.p,OMEGA_2_P_RES);
	omega_2_q = BFP_OF_REAL(stab_att_ref_model_ndi.omega.q*stab_att_ref_model_ndi.omega.q,OMEGA_2_Q_RES);
	omega_2_r = BFP_OF_REAL(stab_att_ref_model_ndi.omega.r*stab_att_ref_model_ndi.omega.r,OMEGA_2_R_RES);

  /* compute reference angular accelerations */
//	  stab_att_ndi_sp_quat.qi = BFP_OF_REAL(0.8253,INT32_QUAT_FRAC);//(int64_t)stab_att_ndi_sp_quat.qi;
//	  stab_att_ndi_sp_quat.qx = BFP_OF_REAL(0.0,INT32_QUAT_FRAC);//(int64_t)stab_att_ndi_sp_quat.qx;
//	  stab_att_ndi_sp_quat.qy = BFP_OF_REAL(0.5646,INT32_QUAT_FRAC);//(int64_t)stab_att_ndi_sp_quat.qy;
//	  stab_att_ndi_sp_quat.qz = BFP_OF_REAL(0.0,INT32_QUAT_FRAC);//(int64_t)stab_att_ndi_sp_quat.qz;
//	  stab_att_ref_quat.qi = BFP_OF_REAL(1,INT32_QUAT_FRAC);//(int64_t)stab_att_ref_quat.qi;
//	  stab_att_ref_quat.qx = BFP_OF_REAL(0,INT32_QUAT_FRAC);//(int64_t)stab_att_ref_quat.qx;
//	  stab_att_ref_quat.qy = BFP_OF_REAL(0,INT32_QUAT_FRAC);//(int64_t)stab_att_ref_quat.qy;
//	  stab_att_ref_quat.qz = BFP_OF_REAL(0,INT32_QUAT_FRAC);//(int64_t)stab_att_ref_quat.qz;

  struct Int32Quat err;
//  struct Int64Quat err_64;
//  struct Int64Quat stab_att_ndi_sp_quat_64;
//  struct Int64Quat stab_att_ref_quat_64;
//  stab_att_ndi_sp_quat_64.qi = BFP_OF_REAL(0.796,INT32_QUAT_FRAC);(int64_t)stab_att_ndi_sp_quat.qi;
//  stab_att_ndi_sp_quat_64.qx = BFP_OF_REAL(0.0,INT32_QUAT_FRAC);(int64_t)stab_att_ndi_sp_quat.qx;
//  stab_att_ndi_sp_quat_64.qy = BFP_OF_REAL(0.652,INT32_QUAT_FRAC);(int64_t)stab_att_ndi_sp_quat.qy;
//  stab_att_ndi_sp_quat_64.qz = BFP_OF_REAL(0.0,INT32_QUAT_FRAC);(int64_t)stab_att_ndi_sp_quat.qz;
//  stab_att_ref_quat_64.qi = BFP_OF_REAL(1,INT32_QUAT_FRAC);(int64_t)stab_att_ref_quat.qi;
//  stab_att_ref_quat_64.qx = BFP_OF_REAL(0,INT32_QUAT_FRAC);(int64_t)stab_att_ref_quat.qx;
//  stab_att_ref_quat_64.qy = BFP_OF_REAL(0,INT32_QUAT_FRAC);(int64_t)stab_att_ref_quat.qy;
//  stab_att_ref_quat_64.qz = BFP_OF_REAL(0,INT32_QUAT_FRAC);(int64_t)stab_att_ref_quat.qz;
  /* compute reference attitude error, use int64 due to large errors        */
  INT32_QUAT_INV_COMP(err, stab_att_ndi_sp_quat, stab_att_ref_quat);
//  (err_64).qi = (((stab_att_ndi_sp_quat_64).qi*(stab_att_ref_quat_64).qi + (stab_att_ndi_sp_quat_64).qx*(stab_att_ref_quat_64).qx + (stab_att_ndi_sp_quat_64).qy*(stab_att_ref_quat_64).qy + (stab_att_ndi_sp_quat_64).qz*(stab_att_ref_quat_64).qz)>>15);
//  (err_64).qx = (((stab_att_ndi_sp_quat_64).qi*(stab_att_ref_quat_64).qx - (stab_att_ndi_sp_quat_64).qx*(stab_att_ref_quat_64).qi - (stab_att_ndi_sp_quat_64).qy*(stab_att_ref_quat_64).qz + (stab_att_ndi_sp_quat_64).qz*(stab_att_ref_quat_64).qy)>>15);
//  (err_64).qy = (((stab_att_ndi_sp_quat_64).qi*(stab_att_ref_quat_64).qy + (stab_att_ndi_sp_quat_64).qx*(stab_att_ref_quat_64).qz - (stab_att_ndi_sp_quat_64).qy*(stab_att_ref_quat_64).qi - (stab_att_ndi_sp_quat_64).qz*(stab_att_ref_quat_64).qx)>>15);
//  (err_64).qz = (((stab_att_ndi_sp_quat_64).qi*(stab_att_ref_quat_64).qz - (stab_att_ndi_sp_quat_64).qx*(stab_att_ref_quat_64).qy + (stab_att_ndi_sp_quat_64).qy*(stab_att_ref_quat_64).qx - (stab_att_ndi_sp_quat_64).qz*(stab_att_ref_quat_64).qi)>>15);
//
//  err.qi = (int32_t)err_64.qi;
//  err.qx = (int32_t)err_64.qx;
//  err.qy = (int32_t)err_64.qy;
//  err.qz = (int32_t)err_64.qz;

  /* wrap it in the shortest direction       */
  INT32_QUAT_WRAP_SHORTEST(err);

  /* propagate the 2nd order linear model    */
  const struct Int64Rates accel_rate = {
  		((((int64_t)(-2*zeta_omega_p)) * ((int64_t)(stab_att_ref_rate.p))) >> (INT64_ATT_REF_RATE_FRAC + ZETA_OMEGA_P_RES - INT64_ATT_REF_ACCEL_FRAC)),
  		((((int64_t)(-2*zeta_omega_q)) * ((int64_t)(stab_att_ref_rate.q))) >> (INT64_ATT_REF_RATE_FRAC + ZETA_OMEGA_P_RES - INT64_ATT_REF_ACCEL_FRAC)),
  		((((int64_t)(-2*zeta_omega_r)) * ((int64_t)(stab_att_ref_rate.r))) >> (INT64_ATT_REF_RATE_FRAC + ZETA_OMEGA_P_RES - INT64_ATT_REF_ACCEL_FRAC)) };

  const struct Int64Rates accel_angle = {
  		((((int64_t)(-omega_2_p)) * ((int64_t)(err.qx))) >> (INT32_QUAT_FRAC + OMEGA_2_P_RES - INT64_ATT_REF_ACCEL_FRAC)),
  		((((int64_t)(-omega_2_q)) * ((int64_t)(err.qy))) >> (INT32_QUAT_FRAC + OMEGA_2_P_RES - INT64_ATT_REF_ACCEL_FRAC)),
  		((((int64_t)(-omega_2_r)) * ((int64_t)(err.qz))) >> (INT32_QUAT_FRAC + OMEGA_2_P_RES - INT64_ATT_REF_ACCEL_FRAC)) };

  RATES_SUM(stab_att_ref_accel, accel_rate, accel_angle);

  /* integrate accelerations */
  const struct Int64Rates delta_rate = {
  		stab_att_ref_accel.p >> ( F_UPDATE_RES + INT64_ATT_REF_ACCEL_FRAC - INT64_ATT_REF_RATE_FRAC),
  		stab_att_ref_accel.q >> ( F_UPDATE_RES + INT64_ATT_REF_ACCEL_FRAC - INT64_ATT_REF_RATE_FRAC),
  		stab_att_ref_accel.r >> ( F_UPDATE_RES + INT64_ATT_REF_ACCEL_FRAC - INT64_ATT_REF_RATE_FRAC)};
  RATES_ADD(stab_att_ref_rate, delta_rate);

  /* set ff term */
  stab_att_ref_ff.p = stab_att_ref_rate.p  >> (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC);
  stab_att_ref_ff.q = stab_att_ref_rate.q  >> (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC);
  stab_att_ref_ff.r = stab_att_ref_rate.r  >> (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC);

//  /*subtract v_h TODO PCH IN PROPER FRAME*/
//  stab_att_ref_rate.p = stab_att_ref_rate.p - (((int64_t)(v_h_att.p)) << (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC));
//  stab_att_ref_rate.q = stab_att_ref_rate.q - (((int64_t)(v_h_att.q)) << (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC));
//  stab_att_ref_rate.r = stab_att_ref_rate.r - (((int64_t)(v_h_att.r)) << (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC));

  /* integrate rate */
  const struct Int64Rates rate_ref_scaled = {
  		OFFSET_AND_ROUND(stab_att_ref_rate.p, (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC)),
  		OFFSET_AND_ROUND(stab_att_ref_rate.q, (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC)),
  		OFFSET_AND_ROUND(stab_att_ref_rate.r, (INT64_ATT_REF_RATE_FRAC - INT32_RATE_FRAC)) };

  struct Int64Quat qdot;
  INT32_QUAT_DERIVATIVE(qdot, rate_ref_scaled, stab_att_ref_quat);
  //QUAT_SMUL(qdot, qdot, DT_UPDATE);
  qdot.qi = qdot.qi >> F_UPDATE_RES;
  qdot.qx = qdot.qx >> F_UPDATE_RES;
  qdot.qy = qdot.qy >> F_UPDATE_RES;
  qdot.qz = qdot.qz >> F_UPDATE_RES;
  QUAT_ADD(stab_att_ref_quat, qdot);
  INT32_QUAT_NORMALIZE(stab_att_ref_quat);

  /* compute ref_euler for debugging and telemetry */
  struct Int32Eulers ref_eul;
  INT32_EULERS_OF_QUAT(ref_eul, stab_att_ref_quat);
  INT32_EULERS_LSHIFT(stab_att_ref_euler, ref_eul, (REF_ANGLE_FRAC - INT32_ANGLE_FRAC));

//  /*Operating time check*/
//  uint32_t time;
//
//  time = sys_time.nb_sec;
//
//  if ((time > 30) && (time <= 100))
//    counter = counter + 1;
//
//  /*Calculation speed check*/
//    uint32_t ticks;
//    uint32_t ticks_diff;
//
//    ticks = sys_time.nb_tick;
//    ticks_diff = ticks - prev_ticks;
//    prev_ticks = ticks;
//
//  /*DEBUG REMOVE*/
//  alt_test1 = ticks_diff;//FLOAT_OF_BFP(stab_att_ref_quat.qy,INT32_QUAT_FRAC);//counter;//stab_att_ref_model_ndi.zeta.q;//
}
