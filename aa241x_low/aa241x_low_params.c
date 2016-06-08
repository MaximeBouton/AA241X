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
 * @file aa241x_low_params.c
 *
 * Definition of custom parameters for low priority controller.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author Maxime Bouton       <boutonm@stanford.edu>
 *  @author Jessie Lauzon       <jessicatlauzon@gmail.com>
 */

#include "aa241x_low_params.h"



/*
 * controller parameters, use max. 15 characters for param name!
 *
 */


/**
*
* @group AA241x Low Param
*/
PARAM_DEFINE_FLOAT(AAL_DESALT,40.0f);


int aal_parameters_init(struct aal_param_handles *h)
{


	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
	

	// TODO: add the above line for each of your custom parameters........

	h->des_alt              =  param_find("AAL_DESALT");
	

	return OK;
}

int aal_parameters_update(const struct aal_param_handles *h, struct aal_params *p)
{

	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
	

	// TODO: add the above line for each of your custom parameters.....

	param_get(h->des_alt, &(p->des_alt));

	

	return OK;
}
