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

struct Int32Vect3 pch_trans_accel;// INT32_RATE_FRAC
struct Int32Rates pch_ang_accel;// INT32_RATE_FRAC

int32_t total_thrust; //INT32_RATE_FRAC
struct Int32Vect3 moments;

int64_t engine_omega[4]; //INT32_RATE_FRAC
int64_t power[4]; //PCH_ENG_POW_FRAC

int16_t limit_cmd[4]; // thrust command with limits

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

	/*TODO: find engine dynamics*/

	/*engine constants*/
	int32_t k_v = BFP_OF_REAL(42.5/600.,INT32_RATE_FRAC); //INT32_RATE_FRAC
	int32_t k_omega = BFP_OF_REAL(0.1,INT32_RATE_FRAC); //INT32_RATE_FRAC
	int32_t k_drag = BFP_OF_REAL(0.1,INT32_RATE_FRAC); //INT32_RATE_FRAC
	int32_t I = BFP_OF_REAL(0.0184,INT32_RATE_FRAC); //INT32_RATE_FRAC

	int64_t moment[4]; //INT32_MOMENT_FRAC
  int8_t i;

  /*update all engines*/
	for (i = 0; i < 4; ++i) {
		/*moment on engine*/
		moment[i] = ((int64_t)k_v*(int64_t)limit_cmd[i] - INT_MULT_RSHIFT((int64_t)k_omega,engine_omega[i],INT32_RATE_FRAC) - INT_MULT_RSHIFT(engine_omega[i],(int64_t)k_drag,INT32_RATE_FRAC));

		/*integrate*/
		engine_omega[i] = engine_omega[i] + (int32_t)(((moment[i] << INT32_RATE_FRAC)/((int64_t)I)) >> F_UPDATE);
		/*TODO: INT*/
		/*calculate power*/
		power[i] = (int64_t)((((pow(((float)(engine_omega[i])/(1<<(12))),3))*5.3*pow(10,(-10))))*((int64_t)1<<(PCH_ENG_POW_FRAC)));
	}

}

/*find rotor forces F and moments M*/
void stabilization_pch_rotor_fm(){

	/*TODO: use engine rpm */
	/*
	 * calculate thrust without engine dynamics TODO: INT
	 */
	struct Int32Thrust thrust_i; // with INT32_STAB_ALT_T_FRAC
  float thrust[4]; //thrust in N

	if (((float)limit_cmd[0])*0.013 > electrical.vsupply)
		thrust[0] = (85./0.85*((float)electrical.vsupply/10.) - 275.)/1000.*9.80665/4.;
	else{
		attitude_t_from_tcommand(&thrust_i.T1, &limit_cmd[0]);
		thrust[0] = FLOAT_OF_BFP(thrust_i.T1,INT32_STAB_ALT_T_FRAC);
	}

	if (((float)limit_cmd[1])*0.013 > electrical.vsupply)
		thrust[1] = (85./0.85*((float)electrical.vsupply/10.) - 275.)/1000.*9.80665/4.;
	else{
		attitude_t_from_tcommand(&thrust_i.T2, &limit_cmd[1]);
		thrust[1] = FLOAT_OF_BFP(thrust_i.T2,INT32_STAB_ALT_T_FRAC);
	}

	if (((float)limit_cmd[2])*0.013 > electrical.vsupply)
		thrust[2] = (85./0.85*((float)electrical.vsupply/10.) - 275.)/1000.*9.80665/4.;
	else{
		attitude_t_from_tcommand(&thrust_i.T3, &limit_cmd[2]);
		thrust[2] = FLOAT_OF_BFP(thrust_i.T3,INT32_STAB_ALT_T_FRAC);
	}

	if (((float)limit_cmd[3])*0.013 > electrical.vsupply)
		thrust[3] = (85./0.85*((float)electrical.vsupply/10.) - 275.)/1000.*9.80665/4.;
	else{
		attitude_t_from_tcommand(&thrust_i.T4, &limit_cmd[3]);
		thrust[3] = FLOAT_OF_BFP(thrust_i.T4,INT32_STAB_ALT_T_FRAC);
	}

//	/*rotor constants*/
//	int32_t A = BFP_OF_REAL(pow(0.10,2)*3.14,INT32_RATE_FRAC);
//	int32_t rho = BFP_OF_REAL(1.225,INT32_RATE_FRAC);
//
//	/* TODO: INT */
//
//	/*rotor thrust*/
//	float thrust[4];
//
//	if (power[0] >= 0)
//	  thrust[0] = pow(((float)(power[0])/((int64_t)1<<(PCH_ENG_POW_FRAC)))*sqrt(2.*FLOAT_OF_BFP(rho*A,2*INT32_RATE_FRAC)),2./3.);
//	else
//    thrust[0] = -pow(((float)(power[0])/((int64_t)1<<(PCH_ENG_POW_FRAC)))*sqrt(2.*FLOAT_OF_BFP(rho*A,2*INT32_RATE_FRAC)),2./3.);
//
//	if (power[1] >= 0)
//	  thrust[1] = pow(((float)(power[1])/((int64_t)1<<(PCH_ENG_POW_FRAC)))*sqrt(2.*FLOAT_OF_BFP(rho*A,2*INT32_RATE_FRAC)),2./3.);
//	else
//    thrust[1] = -pow(((float)(power[1])/((int64_t)1<<(PCH_ENG_POW_FRAC)))*sqrt(2.*FLOAT_OF_BFP(rho*A,2*INT32_RATE_FRAC)),2./3.);
//
//	if (power[2] >= 0)
//	  thrust[2] = pow(((float)(power[2])/((int64_t)1<<(PCH_ENG_POW_FRAC)))*sqrt(2.*FLOAT_OF_BFP(rho*A,2*INT32_RATE_FRAC)),2./3.);
//	else
//    thrust[2] = -pow(((float)(power[2])/((int64_t)1<<(PCH_ENG_POW_FRAC)))*sqrt(2.*FLOAT_OF_BFP(rho*A,2*INT32_RATE_FRAC)),2./3.);
//
//	if (power[3] >= 0)
//	  thrust[3] = pow(((float)(power[3])/((int64_t)1<<(PCH_ENG_POW_FRAC)))*sqrt(2.*FLOAT_OF_BFP(rho*A,2*INT32_RATE_FRAC)),2./3.);
//	else
//    thrust[3] = -pow(((float)(power[3])/((int64_t)1<<(PCH_ENG_POW_FRAC)))*sqrt(2.*FLOAT_OF_BFP(rho*A,2*INT32_RATE_FRAC)),2./3.);

	/*TODO: add torque calc*/
	/*rotor torque*/
	int64_t torque[4]; //PCH_ENG_POW_FRAC
	int64_t total_torque; //PCH_ENG_POW_FRAC

	if (engine_omega[0] > 0)
	  torque[0] = (power[0] << INT32_RATE_FRAC)/engine_omega[0]*0.5;
	else
		torque[0] = 0;

	if (engine_omega[1] > 0)
	  torque[1] = (power[1] << INT32_RATE_FRAC)/engine_omega[1]*0.5;
	else
		torque[1] = 0;

	if (engine_omega[2] > 0)
	  torque[2] = (power[2] << INT32_RATE_FRAC)/engine_omega[2]*0.5;
	else
		torque[2] = 0;

	if (engine_omega[3] > 0)
	  torque[3] = (power[3] << INT32_RATE_FRAC)/engine_omega[3]*0.5;
	else
		torque[3] = 0;

	total_torque = -torque[0] + torque[1] - torque[2] + torque[3];

	/*total forces and moments*/
  total_thrust = -BFP_OF_REAL((thrust[0] + thrust[1] + thrust[2] + thrust[3]),INT32_RATE_FRAC);
	moments.x = BFP_OF_REAL(((thrust[0] + thrust[3] - thrust[1] - thrust[2])*STABILIZATION_NDI_ARM),INT32_STAB_PCH_M_FRAC);
	moments.y = BFP_OF_REAL(((thrust[0] + thrust[1] - thrust[2] - thrust[3])*STABILIZATION_NDI_ARM),INT32_STAB_PCH_M_FRAC);
	moments.z = (int32_t)(total_torque >> (PCH_ENG_POW_FRAC - INT32_STAB_PCH_M_FRAC));

}

/*calculate the body acceleration and angular acceleration*/
void stabilization_pch_body_accel(){

	/*forces in body frame*/
	struct Int32Vect3 forces_body; // INT32_RATE_FRAC
	forces_body.x = 0;
	forces_body.y = 0;
	forces_body.z = total_thrust;

	/*convert to earth frame*/
	struct Int32RMat *r_mat = stateGetNedToBodyRMat_i();
	struct Int32Vect3 forces_earth; // INT32_RATE_FRAC

	INT32_RMAT_TRANSP_VMULT(forces_earth,*r_mat,forces_body);

	/*add gravity*/
	int32_t g_force; // with INT32_RATE_FRAC
	g_force = ((int32_t)(9.80665*mass)) >> (INT32_STAB_ALT_MASS_FRAC - INT32_RATE_FRAC);
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

  /* limit thrust command */
  if (thrust_command[0] < 0)
  	limit_cmd[0] = 0;
  else if (limit_cmd[0] > 9600)
  	limit_cmd[0] = 9600;
  else
  	limit_cmd[0] = thrust_command[0];

  /* limit thrust command */
  if (thrust_command[1] < 0)
  	limit_cmd[1] = 0;
  else if (limit_cmd[1] > 9600)
  	limit_cmd[1] = 9600;
  else
  	limit_cmd[1] = thrust_command[1];

  /* limit thrust command */
  if (thrust_command[2] < 0)
  	limit_cmd[2] = 0;
  else if (limit_cmd[2] > 9600)
  	limit_cmd[2] = 9600;
  else
    limit_cmd[2] = thrust_command[2];

  /* limit thrust command */
  if (thrust_command[3] < 0)
  	limit_cmd[3] = 0;
  else if (limit_cmd[3] > 9600)
  	limit_cmd[3] = 9600;
  else
    limit_cmd[3] = thrust_command[3];

	/*update engine speed*/
	stabilization_pch_update_engine();

	/*calculate rotor forces and moments*/
  stabilization_pch_rotor_fm();

  /*calculate body accelerations*/
  stabilization_pch_body_accel();

}

