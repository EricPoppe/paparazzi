/*
 * stabilization_attitude_ndi_pch_model.h
 *
 *  Created on: 1 mei 2014
 *      Author: cpoppe
 */

#ifndef STABILIZATION_ATTITUDE_NDI_PCH_MODEL_H_
#define STABILIZATION_ATTITUDE_NDI_PCH_MODEL_H_

extern struct Int32Vect3 pch_trans_accel;
extern struct Int32Rates pch_ang_accel;

extern void stabilization_pch_update(void);
extern void stabilization_pch_init(void);

void stabilization_pch_update_engine(void);
void stabilization_pch_rotor_fm(void);
void stabilization_pch_body_accel(void);

#endif /* STABILIZATION_ATTITUDE_NDI_PCH_MODEL_H_ */
