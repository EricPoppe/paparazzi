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
 * @file firmwares/rotorcraft/stabilization/stabilization_rate_ref_int.h
 *
 * Rotorcraft rate reference generation.
 * (int version)
 *
 */

#ifndef STABILIZATION_RATE_INT_REF_H
#define STABILIZATION_RATE_INT_REF_H

#include "stabilization_rate_ref_int.h"
#include "stabilization_rate_ndi_int.h"
#include "math/pprz_algebra_int.h"
#include "stabilization_attitude_ndi_pch_model.h"

#define RATE_REF_JERK_FRAC 18
#define RATE_REF_ACCEL_FRAC 27
#define RATE_REF_RATE_FRAC 36

extern struct Int64Rates  stab_rate_ref_accel; ///< with RATE_REF_ACCEL_FRAC
extern struct Int64Rates  stab_rate_ref_jerk; ///< with RATE_REF_JERK_FRAC
extern struct Int32Rates  stab_rate_sp;  ///< with INT32_RATE_FRAC
extern struct Int64Rates  stab_rate_ref;  ///< with RATE_REF_RATE_FRAC

struct Int32RateRefModel {
  struct Int32Rates omega;
  struct Int32Rates zeta;
};

extern struct Int32RateRefModel stab_rate_ref_model;

//void stabilization_rate_ref_enter(void);
extern void stabilization_rate_ref_init(void);
extern void stabilization_rate_ref_update(void);

#endif /* STABILIZATION_RATE_INT_REF_H */
