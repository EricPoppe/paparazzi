/*
 * stabilization_attitude_ndi_pch_model.c
 *
 *  Created on: 1 mei 2014
 *      Author: cpoppe
 */

#include "stabilization_attitude_ndi_pch_model.h"
#include "stabilization_attitude_ndi_quat_int.h"
#include "state.h"

#include <stdio.h>
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra.h"
#include "state.h"
#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/electrical.h"

#include "std.h"
#include <stdlib.h>
#include <math.h>

#define F_UPDATE 9
#define PCH_ENG_POW_FRAC 40
#define INT32_STAB_PCH_M_FRAC 20

#ifndef STABILIZATION_NDI_ARM
#define STABILIZATION_NDI_ARM 0.126
#endif

struct Int32Vect3 pch_trans_accel;// INT32_PCH_F_FRAC
struct Int32Rates pch_ang_accel;// INT32_RATE_FRAC

int32_t total_thrust; //INT32_PCH_T_FRAC
struct Int32Vect3 moments;

int64_t engine_omega[4]; //INT32_PCH_OMEGA_FRAC
int64_t power[4]; //PCH_ENG_POW_FRAC

int16_t pch_cmd[4]; // thrust command with limits

/* set all the states of the PCH to zero*/
void stabilization_pch_init(){

	pch_trans_accel.x = 0;
	pch_trans_accel.y = 0;
	pch_trans_accel.z = 0;

	pch_ang_accel.p = 0;
	pch_ang_accel.q = 0;
	pch_ang_accel.r = 0;

	engine_omega[0] = 0;
	engine_omega[1] = 0;
	engine_omega[2] = 0;
	engine_omega[3] = 0;

	power[0] = 0;
	power[1] = 0;
	power[2] = 0;
	power[3] = 0;

}

/* calculate and integrate the engine angular acceleration */
void stabilization_pch_update_engine(){

	/*engine constants*/
	int32_t k_u = BFP_OF_REAL(0.05229,INT32_PCH_C_RPM_FRAC); //INT32_C_RPM_FRAC
	int32_t k_u_tau_up = BFP_OF_REAL(3.486,INT32_PCH_OMEGA_FRAC); //INT32_PCH_OMEGA_FRAC
	int32_t k_u_tau_down = BFP_OF_REAL(1.494,INT32_PCH_OMEGA_FRAC); //INT32_PCH_OMEGA_FRAC
	int32_t tau_up_1 = BFP_OF_REAL(66.67,INT32_PCH_OMEGA_FRAC); //INT32_PCH_OMEGA_FRAC
	int32_t tau_down_1 = BFP_OF_REAL(28.57,INT32_PCH_OMEGA_FRAC); //INT32_PCH_OMEGA_FRAC

	int64_t omega_d[4]; //INT32_PCH_OMEGA_D_FRAC
	int8_t i;

	/*update all engines*/
	for (i = 0; i < 4; ++i) {
		/*check if spinning up or down*/
		if (engine_omega[i] <= (k_u*pch_cmd[i] >> (INT32_PCH_C_RPM_FRAC - INT32_PCH_OMEGA_FRAC))){
			omega_d[i] = (((int64_t)k_u_tau_up)*((int64_t)pch_cmd[i]) >> (INT32_PCH_OMEGA_FRAC - INT32_PCH_OMEGA_D_FRAC))
							- (((int64_t)tau_up_1)*engine_omega[i] >> (2*INT32_PCH_OMEGA_FRAC - INT32_PCH_OMEGA_D_FRAC));
		}
		else{
			omega_d[i] = (((int64_t)k_u_tau_down)*((int64_t)pch_cmd[i]) >> (INT32_PCH_OMEGA_FRAC - INT32_PCH_OMEGA_D_FRAC))
							- (((int64_t)tau_down_1)*engine_omega[i] >> (2*INT32_PCH_OMEGA_FRAC - INT32_PCH_OMEGA_D_FRAC));
		}
		/*integrate*/
		engine_omega[i] = engine_omega[i] + (omega_d[i] >> (F_UPDATE + INT32_PCH_OMEGA_D_FRAC - INT32_PCH_OMEGA_FRAC));

		/*limit to max rpm of 665 and min of 180*/
		if (engine_omega[i] > BFP_OF_REAL(665,INT32_PCH_OMEGA_FRAC)){
			engine_omega[i] = BFP_OF_REAL(665,INT32_PCH_OMEGA_FRAC);
		}
		else if (engine_omega[i] < BFP_OF_REAL(180,INT32_PCH_OMEGA_FRAC)){
			engine_omega[i] = BFP_OF_REAL(180,INT32_PCH_OMEGA_FRAC);
		}
	}
}

/*find rotor forces F and moments M*/
void stabilization_pch_rotor_fm(){

	/*rotor thrust and torque */
	int64_t engine_omega_2; //INT32_PCH_OMEGA_FRAC
	int64_t thrust[4]; // INT32_PCH_T_FRAC
	int64_t torque[4]; //INT32_PCH_T_FRAC
	int64_t total_torque; //PCH_ENG_POW_FRAC
	int64_t C_T_2 = ((0.0000051485)*((int64_t)1<<(INT64_PCH_C_T_FRAC))); //INT64_PCH_C_T_FRAC
	int64_t C_T = ((0.0005651)*((int64_t)1<<(40))); //INT64_PCH_C_T_FRAC
	int32_t C_T_c = BFP_OF_REAL(0.04949,INT32_PCH_T_FRAC); //INT32_PCH_T_FRAC
	int8_t i;

	for (i = 0; i < 4; ++i){
		engine_omega_2 = INT_MULT_RSHIFT(engine_omega[i],engine_omega[i],INT32_PCH_OMEGA_FRAC);
		thrust[i] = (C_T_2*engine_omega_2 >> ((INT64_PCH_C_T_FRAC + INT32_PCH_OMEGA_FRAC)	- INT32_PCH_T_FRAC))
						- (C_T*engine_omega[i] >> ((INT64_PCH_C_T_FRAC + INT32_PCH_OMEGA_FRAC) - INT32_PCH_T_FRAC))
						+ C_T_c;
		torque[i] = 0.0125*thrust[i];
	}

	total_torque = -torque[0] + torque[1] - torque[2] + torque[3];

	/*total forces and moments*/
	total_thrust = thrust[0] + thrust[1] + thrust[2] + thrust[3];
	moments.x = ((int64_t)((thrust[0] + thrust[3] - thrust[1] - thrust[2])*STABILIZATION_NDI_ARM)) >> (INT32_PCH_T_FRAC - INT32_STAB_PCH_M_FRAC);
	moments.y = ((int64_t)((thrust[0] + thrust[1] - thrust[2] - thrust[3])*STABILIZATION_NDI_ARM)) >> (INT32_PCH_T_FRAC - INT32_STAB_PCH_M_FRAC);
	moments.z = (int32_t)(total_torque >> (INT32_PCH_T_FRAC - INT32_STAB_PCH_M_FRAC));

}

/* calculate the body acceleration and angular acceleration */
void stabilization_pch_body_accel(){

	/*forces in body frame*/
	struct Int32Vect3 forces_body; // INT32_STAB_ALT_T_FRAC
	forces_body.x = 0;
	forces_body.y = 0;
	forces_body.z = -((int32_t)(total_thrust >> (INT32_PCH_T_FRAC - INT32_PCH_F_FRAC)));

	/*convert to earth frame*/
	struct Int32RMat *r_mat = stateGetNedToBodyRMat_i();
	struct Int32Vect3 forces_earth; // INT32_PCH_F_FRAC

	INT32_RMAT_TRANSP_VMULT(forces_earth,*r_mat,forces_body);

	/*add gravity*/
	int32_t g_force; // with INT32_PCH_F_FRAC
	g_force = ((int32_t)(9.80665*mass)) >> (INT32_STAB_ALT_MASS_FRAC - INT32_PCH_F_FRAC);
	forces_earth.z = forces_earth.z + g_force;

	/*accelerations in earth frame*/
	if (mass > 0){
		pch_trans_accel.x = (int32_t)(((int64_t)forces_earth.x << INT32_STAB_ALT_MASS_FRAC)/mass);
		pch_trans_accel.y = (int32_t)(((int64_t)forces_earth.y << INT32_STAB_ALT_MASS_FRAC)/mass);
		pch_trans_accel.z = (int32_t)(((int64_t)forces_earth.z << INT32_STAB_ALT_MASS_FRAC)/mass);
	}
	else {
		pch_trans_accel.x = 0;
		pch_trans_accel.y = 0;
		pch_trans_accel.z = 0;
	}

	/*angular accelerations in body frame TODO: INT*/
	struct FloatMat33 I_mat;
	struct FloatMat33 I_mat_inv;
	FLOAT_MAT33_DIAG(I_mat, STABILIZATION_NDI_IXX, STABILIZATION_NDI_IYY, STABILIZATION_NDI_IZZ);
	FLOAT_MAT33_DIAG(I_mat_inv, 1/STABILIZATION_NDI_IXX, 1/STABILIZATION_NDI_IYY, 1/STABILIZATION_NDI_IZZ);

	struct FloatVect3 omega_body;
	struct FloatVect3 Iomega;
	struct FloatVect3 omegaIomega;
	struct FloatVect3 moment_omegaIomega;

	omega_body.x = stateGetBodyRates_f()->p;
	omega_body.y = stateGetBodyRates_f()->q;
	omega_body.z = stateGetBodyRates_f()->r;

	MAT33_VECT3_MUL(Iomega,I_mat,omega_body);
	VECT3_CROSS_PRODUCT(omegaIomega,omega_body,Iomega);

	struct FloatVect3 omega_d_body;
	struct FloatVect3 moments_f;
	moments_f.x = FLOAT_OF_BFP(moments.x,INT32_STAB_PCH_M_FRAC);
	moments_f.y = FLOAT_OF_BFP(moments.y,INT32_STAB_PCH_M_FRAC);
	moments_f.z = FLOAT_OF_BFP(moments.z,INT32_STAB_PCH_M_FRAC);

	moment_omegaIomega.x = moments_f.x - omegaIomega.x;
	moment_omegaIomega.y = moments_f.y - omegaIomega.y;
	moment_omegaIomega.z = moments_f.z - omegaIomega.z;

	MAT33_VECT3_MUL(omega_d_body,I_mat_inv,moment_omegaIomega);

	pch_ang_accel.p = BFP_OF_REAL(omega_d_body.x,INT32_RATE_FRAC);
	pch_ang_accel.q = BFP_OF_REAL(omega_d_body.y,INT32_RATE_FRAC);
	pch_ang_accel.r = BFP_OF_REAL(omega_d_body.z,INT32_RATE_FRAC);

}

void stabilization_pch_update(){

	/* limit thrust command and add minimum command */
	if (thrust_command[0] < 0)
		pch_cmd[0] = 3471;
	else if (thrust_command[0] > 9600)
		pch_cmd[0] = 3471 + 9600;
	else
		pch_cmd[0] = 3471 + thrust_command[0];

	if (thrust_command[1] < 0)
		pch_cmd[1] = 3471;
	else if (thrust_command[1] > 9600)
		pch_cmd[1] = 3471 + 9600;
	else
		pch_cmd[1] = 3471 + thrust_command[1];

	if (thrust_command[2] < 0)
		pch_cmd[2] = 3471;
	else if (thrust_command[2] > 9600)
		pch_cmd[2] = 3471 + 9600;
	else
		pch_cmd[2] = 3471 + thrust_command[2];

	if (thrust_command[3] < 0)
		pch_cmd[3] = 3471;
	else if (thrust_command[3] > 9600)
		pch_cmd[3] = 3471 + 9600;
	else
		pch_cmd[3] = 3471 + thrust_command[3];

//	/*DEBUG REMOVE*/
//	pch_cmd[0] = 3471+6428;
//	pch_cmd[1] = 3471+6428;
//	pch_cmd[2] = 3471+6428;
//	pch_cmd[3] = 3471+6428;

	/*update engine speed*/
	stabilization_pch_update_engine();

	/*calculate rotor forces and moments*/
	stabilization_pch_rotor_fm();

	/*calculate body accelerations*/
	stabilization_pch_body_accel();

}

