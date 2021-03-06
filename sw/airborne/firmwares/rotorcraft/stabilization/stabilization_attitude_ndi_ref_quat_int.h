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
 * @file firmwares/rotorcraft/stabilization/stabilization_attitude_ndi_ref_quat_int.h
 *
 * Rotorcraft attitude reference generation.
 * (quaternion int version)
 *
 */

#ifndef STABILIZATION_ATTITUDE_INT_NDI_REF_QUAT_INT_H
#define STABILIZATION_ATTITUDE_INT_NDI_REF_QUAT_INT_H

#include "stabilization_attitude_ref_int.h"

struct Int32RefModel {
	struct Int32Rates omega;
	struct Int32Rates zeta;
};

struct FloatRatesNDI {
  float p; ///< in rad/s^2
  float q; ///< in rad/s^2
  float r; ///< in rad/s^2
};

struct FloatRefModelNDI {
  struct FloatRatesNDI omega;
  struct FloatRatesNDI zeta;
};

extern struct Int32Quat   stab_att_ndi_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Quat   stab_att_ref_quat;  ///< with #INT32_QUAT_FRAC
extern struct FloatRefModelNDI stab_att_ref_model_ndi;

#define INT64_ATT_REF_QUAT_FRAC 30
#define INT64_ATT_REF_RATE_FRAC 21
#define INT64_ATT_REF_ACCEL_FRAC 12

void stabilization_attitude_ref_enter(void);

#endif /* STABILIZATION_ATTITUDE_INT_NDI_REF_QUAT_INT_H */
