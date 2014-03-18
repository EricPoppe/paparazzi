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

extern struct Int32Quat   stab_rate_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Quat   stab_rate_ref_quat;  ///< with #INT32_QUAT_FRAC

void stabilization_rate_ref_enter(void);

#endif /* STABILIZATION_RATE_INT_REF_H */
