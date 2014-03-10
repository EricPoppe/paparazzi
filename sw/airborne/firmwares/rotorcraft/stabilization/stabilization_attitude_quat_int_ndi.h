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

#ifndef STABILIZATION_ATTITUDE_QUAT_INT_NDI_H
#define STABILIZATION_ATTITUDE_QUAT_INT_NDI_H

//#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

#include "math/pprz_algebra_int.h"

#include "generated/airframe.h"

extern int32_t stabilization_att_fb_cmd[COMMANDS_NB];
extern int32_t stabilization_att_ff_cmd[COMMANDS_NB];

/* Definition of gain structs */

struct att_directions {
	int32_t tilt;
	int32_t yaw;
};

struct rate_directions {
	int32_t p;
	int32_t q;
	int32_t r;
};

struct Int32NDIAttitudeGains {
	struct att_directions p;
	struct att_directions d;
};

struct Int32NDIRateGains {
	struct rate_directions p;
	struct rate_directions i;
	struct rate_directions d;
};

extern struct Int32NDIAttitudeGains  attitude_gains;
extern struct Int32NDIRateGains rate_gains;

#endif /* STABILIZATION_ATTITUDE_QUAT_INT_NDI_H */
