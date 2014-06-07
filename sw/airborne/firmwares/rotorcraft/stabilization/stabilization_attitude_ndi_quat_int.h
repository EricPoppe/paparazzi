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

#ifndef STABILIZATION_ATTITUDE_NDI_QUAT_INT_H
#define STABILIZATION_ATTITUDE_NDI_QUAT_INT_H

//#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ndi_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate_ref_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate_ndi_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_altitude_ndi_quat_int.h"

#include "firmwares/rotorcraft/autopilot.h"

#include "math/pprz_algebra_int.h"
#include "paparazzi.h"

#include "generated/airframe.h"

/*REMOVE DEBUG*/
extern float rate_test;
extern float alt_test1;
extern float alt_test2;

/* Definition of gain structs */

struct att_directions {
	int32_t tilt;
	int32_t yaw;
};

struct Int32NDIAttitudeGains {
	struct att_directions p;
	struct att_directions d;
};

struct Int16Thrust {
	int16_t T1;
	int16_t T2;
	int16_t T3;
	int16_t T4;
};

struct Int32Thrust {
	int32_t T1;
	int32_t T2;
	int32_t T3;
	int32_t T4;
};

extern struct Int32NDIAttitudeGains  attitude_ndi_gains;
extern void stabilization_attitude_thrust_run(bool_t motors_on);
extern int16_t thrust_command[4];
extern void attitude_tcommand_from_t(int16_t *tcom, int32_t *t);
extern void attitude_t_from_tcommand(int32_t *t, int16_t *tcom);
extern int32_t getMaxTavg(void);
extern void attitude_tdiff_from_tau_command(int32_t *tdiff, int32_t *tau_des);

extern int32_t attitude_t_avg_cmd;
extern float psi_f;

extern struct Int32Rates v_h_att;

/*DEBUG REMOVE*/
extern bool_t stabilization_override_on;
extern float phi_sp;
extern float theta_sp;
extern float psi_sp;
extern float phi_d_sp;
extern float theta_d_sp;
extern float psi_d_sp;
extern float z_sp;
extern float z_d_sp;
extern float tdiff_yaw_sp;

extern bool_t tau_step;
extern bool_t att_sp;
extern bool_t att_d_sp;
extern bool_t alt_sp;
extern bool_t alt_d_sp;


#endif /* STABILIZATION_ATTITUDE_NDI_QUAT_INT_H */
