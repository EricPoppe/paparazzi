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

#ifndef STABILIZATION_RATE_NDI_INT_H
#define STABILIZATION_RATE_NDI_INT_H

//#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate_ref_int.h"

#include "math/pprz_algebra_int.h"

#include "generated/airframe.h"

extern int32_t stabilization_att_fb_cmd[COMMANDS_NB];
extern int32_t stabilization_att_ff_cmd[COMMANDS_NB];
extern float test;

extern void stabilization_rate_ndi_run(bool_t in_flight);
extern void stabilization_rate_ndi_init();

/* Definition of structs */

struct rate_directions {
	int32_t p;
	int32_t q;
	int32_t r;
};

struct Int32NDIRateGains {
	struct rate_directions p;
	struct rate_directions i;
	struct rate_directions d;
	struct rate_directions dd;
};

extern struct Int32NDIRateGains rate_ndi_gains;

struct Int32VirtualInput {
	int32_t p;
	int32_t q;
	int32_t r;
};

struct Int32Thrust {
	int32_t T1;
	int32_t T2;
	int32_t T3;
	int32_t T4;
};

#endif /* STABILIZATION_RATE_NDI_QUAT_INT_H */