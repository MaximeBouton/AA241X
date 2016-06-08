/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file aa241x_fw_control_params.c
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author Maxime Bouton       <boutonm@stanford.edu>
 *  @author Jessie Lauzon       <jessicatlauzon@gmail.com>
 */

#include "aa241x_high_params.h"



/*
 *  controller parameters, use max. 15 characters for param name!
 *
 */

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_EXAMPLE and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 */
//PARAM_DEFINE_FLOAT(AAH_EXAMPLE, 10.0f);

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 */
//PARAM_DEFINE_FLOAT(AAH_PROPROLLGAIN, 1.0f);

// TODO: define custom parameters here



/**
* TRIM PARAMETERS
*/
// throttle trim
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_TRTHRO, 0.0f) ;

/*
* @group AA241x High Params
*/
// elevator trim
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_TRELEV, 0.0f);

// rudder
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_TRRUD,0.0f);

// ailerons
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_TRAIL,0.0f);


/**
* CASE NUMBER FOR SEPARATED CONTROL LAW
*/
/*
* @group AA241x High Params
*/
PARAM_DEFINE_INT32(AAH_CASENO, 3);


/**
* LONGITUDINAL CONTROL PARAMETERS
* 
*/
// Gain for the airspeed control loop
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_KU, 1.0f);

// Gain for the pitch inner loop
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_KTHETA, -1.0f);

// Gain for the altitude outer loop
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_KH, 1.0f);



/**
* LATERAL CONTROL PARAMETERS
*
*/

// Gain for roll  inner loop
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_KPHI, 1.0f);

// Gain from Delta_psi to phi_command
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_KPSI, 1.0f);

// Gain for rudder control
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_KRUD, 1.0f);

// Gain for line track 
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_KD, 1.0f);

// des speed 
/*
* @group AA241x High Params
*/
PARAM_DEFINE_FLOAT(AAH_DS, 19.0f);



int aah_parameters_init(struct aah_param_handles *h)
{

	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
	
	//h->example_high_param		= param_find("AAH_EXAMPLE");
	//h->proportional_roll_gain 	= param_find("AAH_PROPROLLGAIN");

	// TODO: add the above line for each of your custom parameters........

	h->throttle_trim = param_find("AAH_TRTHRO");
	h->elevator_trim = param_find("AAH_TRELEV");
	h->rudder_trim   = param_find("AAH_TRRUD");
	h->ailerons_trim = param_find("AAH_TRAIL");

	h->case_no = param_find("AAH_CASENO");

	h->K_u     = param_find("AAH_KU");
	h->K_th    = param_find("AAH_KTHETA");
	h->K_h     = param_find("AAH_KH");
		
	h->K_phi   = param_find("AAH_KPHI");
	h->K_psi   = param_find("AAH_KPSI");
	h->K_rud   = param_find("AAH_KRUD");
	h->K_d     = param_find("AAH_KD");
	
	h->des_sp  = param_find("AAH_DS");
	

	return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{

	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
	//param_get(h->example_high_param, &(p->example_high_param));
	//param_get(h->proportional_roll_gain, &(p->proportional_roll_gain));

	
	
	// TODO: add the above line for each of your custom parameters.....
	param_get(h->throttle_trim, &(p->throttle_trim));
	param_get(h->elevator_trim, &(p->elevator_trim));
	param_get(h->rudder_trim, &(p->rudder_trim));
	param_get(h->ailerons_trim, &(p->ailerons_trim));

	param_get(h->case_no, &(p->case_no));

	param_get(h->K_u, &(p->K_u));
	param_get(h->K_th, &(p->K_th));
	param_get(h->K_h, &(p->K_h));

	param_get(h->K_phi, &(p->K_phi));
	param_get(h->K_psi, &(p->K_psi));
	param_get(h->K_rud, &(p->K_rud));
	param_get(h->K_d, &(p->K_d));
	
	param_get(h->des_sp, &(p->des_sp));

	return OK;
}
