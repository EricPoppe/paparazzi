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
 * @file firmwares/rotorcraft/stabilization/stabilization_rate_ref_int.c
 *
 * Rotorcraft rate reference generation.
 * (int version)
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate_ref_int.h"
#include <stdio.h>
//#include "firmwares/rotorcraft/stabilization/stabilization_rate_ref_saturate.h"

#define REF_ACCEL_MAX_P BFP_OF_REAL(STABILIZATION_RATE_REF_MAX_PDOT, REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_Q BFP_OF_REAL(STABILIZATION_RATE_REF_MAX_QDOT, REF_ACCEL_FRAC)
#define REF_ACCEL_MAX_R BFP_OF_REAL(STABILIZATION_RATE_REF_MAX_RDOT, REF_ACCEL_FRAC)

#define REF_RATE_MAX_P BFP_OF_REAL(STABILIZATION_RATE_REF_MAX_P, REF_RATE_FRAC)
#define REF_RATE_MAX_Q BFP_OF_REAL(STABILIZATION_RATE_REF_MAX_Q, REF_RATE_FRAC)
#define REF_RATE_MAX_R BFP_OF_REAL(STABILIZATION_RATE_REF_MAX_R, REF_RATE_FRAC)

#define OMEGA_P   STABILIZATION_RATE_NDI_REF_OMEGA_P
#define ZETA_P    STABILIZATION_RATE_NDI_REF_ZETA_P
#define ZETA_OMEGA_P_RES 10
#define ZETA_OMEGA_P BFP_OF_REAL((ZETA_P*OMEGA_P), ZETA_OMEGA_P_RES)
#define OMEGA_2_P_RES 7
#define OMEGA_2_P    BFP_OF_REAL((OMEGA_P*OMEGA_P), OMEGA_2_P_RES)

#define OMEGA_Q   STABILIZATION_RATE_NDI_REF_OMEGA_Q
#define ZETA_Q    STABILIZATION_RATE_NDI_REF_ZETA_Q
#define ZETA_OMEGA_Q_RES 10
#define ZETA_OMEGA_Q BFP_OF_REAL((ZETA_Q*OMEGA_Q), ZETA_OMEGA_Q_RES)
#define OMEGA_2_Q_RES 7
#define OMEGA_2_Q    BFP_OF_REAL((OMEGA_Q*OMEGA_Q), OMEGA_2_Q_RES)

#define OMEGA_R   STABILIZATION_RATE_NDI_REF_OMEGA_R
#define ZETA_R    STABILIZATION_RATE_NDI_REF_ZETA_R
#define ZETA_OMEGA_R_RES 10
#define ZETA_OMEGA_R BFP_OF_REAL((ZETA_R*OMEGA_R), ZETA_OMEGA_R_RES)
#define OMEGA_2_R_RES 7
#define OMEGA_2_R    BFP_OF_REAL((OMEGA_R*OMEGA_R), OMEGA_2_R_RES)

struct Int32Rates  stab_rate_sp;          //< with INT32_RATE_FRAC
struct Int64Rates  stab_rate_ref;         //< with RATE_REF_RATE_FRAC
struct Int64Rates  stab_rate_ref_accel;   //< with RATE_REF_ACCEL_FRAC
struct Int64Rates  stab_rate_ref_jerk;    //< with RATE_REF_JERK_FRAC

struct Int32RateRefModel stab_rate_ref_model = {
  {STABILIZATION_RATE_NDI_REF_OMEGA_P, STABILIZATION_RATE_NDI_REF_OMEGA_Q, STABILIZATION_RATE_NDI_REF_OMEGA_R},
  {STABILIZATION_RATE_NDI_REF_ZETA_P, STABILIZATION_RATE_NDI_REF_ZETA_Q, STABILIZATION_RATE_NDI_REF_ZETA_R}
};

void stabilization_rate_ref_init(void) {

  INT_RATES_ZERO(stab_rate_ref);
  INT_RATES_ZERO(stab_rate_ref_accel);
  INT_RATES_ZERO(stab_rate_ref_jerk);

}

/*
 * Reference
 */
#define DT_UPDATE (1./PERIODIC_FREQUENCY)
// CAUTION! Periodic frequency is assumed to be 512 Hz
// which is equal to >> 9
#define F_UPDATE_RES 9

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))

void stabilization_rate_ref_update(void) {

  /* integrate reference acceleration            */
  const struct Int64Rates delta_rate = {
         stab_rate_ref_accel.p >> ( F_UPDATE_RES + RATE_REF_ACCEL_FRAC - RATE_REF_RATE_FRAC),
         stab_rate_ref_accel.q >> ( F_UPDATE_RES + RATE_REF_ACCEL_FRAC - RATE_REF_RATE_FRAC),
         stab_rate_ref_accel.r >> ( F_UPDATE_RES + RATE_REF_ACCEL_FRAC - RATE_REF_RATE_FRAC)};
  RATES_ADD(stab_rate_ref, delta_rate);

  /* integrate reference jerk            */
  const struct Int64Rates delta_accel = {
         stab_rate_ref_jerk.p >> ( F_UPDATE_RES + RATE_REF_JERK_FRAC - RATE_REF_ACCEL_FRAC),
         stab_rate_ref_jerk.q >> ( F_UPDATE_RES + RATE_REF_JERK_FRAC - RATE_REF_ACCEL_FRAC),
         stab_rate_ref_jerk.r >> ( F_UPDATE_RES + RATE_REF_JERK_FRAC - RATE_REF_ACCEL_FRAC)};
  RATES_ADD(stab_rate_ref_accel, delta_accel);

  /*
   * compute reference angular jerk
   */

  /* compute reference rate error        */
  struct Int64Rates err; // with RATE_REF_RATE_FRAC
  struct Int64Rates stab_rate_sp_scaled; // with RATE_REF_RATE_FRAC
  stab_rate_sp_scaled.p = ((int64_t)stab_rate_sp.p) << (RATE_REF_RATE_FRAC - INT32_RATE_FRAC);
  stab_rate_sp_scaled.q = ((int64_t)stab_rate_sp.q) << (RATE_REF_RATE_FRAC - INT32_RATE_FRAC);
  stab_rate_sp_scaled.r = ((int64_t)stab_rate_sp.r) << (RATE_REF_RATE_FRAC - INT32_RATE_FRAC);
  RATES_DIFF(err, stab_rate_ref, stab_rate_sp_scaled);

  /* propagate the 2nd order linear model    */
  const struct Int64Rates jerk_accel = {
    ((int64_t)(-2.*ZETA_OMEGA_P) * (stab_rate_ref_accel.p >> (RATE_REF_ACCEL_FRAC - RATE_REF_JERK_FRAC))) >> (ZETA_OMEGA_P_RES),
    ((int64_t)(-2.*ZETA_OMEGA_Q) * (stab_rate_ref_accel.q >> (RATE_REF_ACCEL_FRAC - RATE_REF_JERK_FRAC))) >> (ZETA_OMEGA_Q_RES),
    ((int64_t)(-2.*ZETA_OMEGA_R) * (stab_rate_ref_accel.r >> (RATE_REF_ACCEL_FRAC - RATE_REF_JERK_FRAC))) >> (ZETA_OMEGA_R_RES) };

  const struct Int64Rates jerk_rate = {
    ((int64_t)(-OMEGA_2_P) * (err.p >> (RATE_REF_RATE_FRAC - RATE_REF_JERK_FRAC))) >> (OMEGA_2_P_RES),
    ((int64_t)(-OMEGA_2_Q) * (err.q >> (RATE_REF_RATE_FRAC - RATE_REF_JERK_FRAC))) >> (OMEGA_2_Q_RES),
    ((int64_t)(-OMEGA_2_R) * (err.r >> (RATE_REF_RATE_FRAC - RATE_REF_JERK_FRAC))) >> (OMEGA_2_R_RES) };

  RATES_SUM(stab_rate_ref_jerk, jerk_accel, jerk_rate);

}
