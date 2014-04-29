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

#ifndef STABILIZATION_ALTITUDE_NDI_QUAT_INT_H
#define STABILIZATION_ALTITUDE_NDI_QUAT_INT_H

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ndi_quat_int.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "paparazzi.h"

#include "math/pprz_algebra_int.h"

#include "generated/airframe.h"

extern struct Int32Quat altitude_attitude_sp;
extern int32_t altitude_t_avg; // average thrust commanded by altitude controller, INT32_STAB_ALT_T_FRAC

//#define GAIN_PRESCALER_SZD_P 1
//#define GAIN_PRESCALER_SZD_I 1
//#define GAIN_PRESCALER_SZD_D 1
//
//#define GAIN_PRESCALER_SZ_P 1
//#define GAIN_PRESCALER_SZ_I 1
//#define GAIN_PRESCALER_SZ_D 1
//
//#define GAIN_PRESCALER_LZD_P 1
//#define GAIN_PRESCALER_LZD_I 1
//#define GAIN_PRESCALER_LZD_D 1
//
//#define GAIN_PRESCALER_LZ_P 1
//#define GAIN_PRESCALER_LZ_I 1
//#define GAIN_PRESCALER_LZ_D 1

/* Definition of gain structs */

struct Int32NDIAltitudeGains {
	int32_t p;
	int32_t i;
	int32_t d;
	int32_t ff;
};

extern struct Int32NDIAltitudeGains small_inner_gains;
extern struct Int32NDIAltitudeGains small_outer_gains;
extern struct Int32NDIAltitudeGains large_inner_gains;
extern struct Int32NDIAltitudeGains large_outer_gains;

/* functions called from attitude stabilization */
extern void stabilization_altitude_init(void);
extern void stabilization_altitude_run(bool_t enable_integrator);

/* setpoints set by vertical guidance */
extern int64_t altitude_z_sp; //with INT64_STAB_ALT_X_REF_FRAC
extern int64_t altitude_zd_sp; //with INT64_STAB_ALT_X_REF_FRAC
extern int8_t altitude_vert_mode; // 0 manual, 1 climb, 2 alt

/* General reference model structs */

struct Int32AltRefModel {
	int32_t omega;
	int32_t zeta;
	int32_t zeta_omega;
	int32_t omega_2;
};

struct IntAltRefModelState {
  int64_t stab_alt_xdd_ref; // with INT64_STAB_ALT_XDD_REF_FRAC
  int64_t stab_alt_xd_ref; // with INT64_STAB_ALT_XD_REF_FRAC
  int64_t stab_alt_x_ref; // with INT64_STAB_ALT_X_REF_FRAC
};

/* General reference model fracs */
#define INT64_STAB_ALT_XDD_REF_FRAC 18
#define INT64_STAB_ALT_XD_REF_FRAC 27
#define INT64_STAB_ALT_X_REF_FRAC 36

/* Controller fracs */
#define INT64_STAB_ALT_Z_FRAC 36
#define INT64_STAB_ALT_ZD_FRAC 27
#define INT64_STAB_ALT_ZDD_FRAC 18

/* Dynamic inversion fracs */
#define INT32_STAB_ALT_MASS_FRAC 20
#define INT32_STAB_ALT_INERT_FRAC 20
#define INT32_STAB_ALT_T_FRAC 16

/* reference models */
extern struct Int32AltRefModel small_inner_ref_model;
extern struct Int32AltRefModel small_outer_ref_model;
extern struct Int32AltRefModel large_inner_ref_model;
extern struct Int32AltRefModel large_outer_ref_model;

extern struct Int32Quat stab_att_sp_quat;

#endif /* STABILIZATION_ALTITUDE_NDI_QUAT_INT_H */
