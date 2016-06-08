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
 * @file aa241x_low.cpp
 *
 * Secondary control law file for AA241x.  Contains control/navigation
 * logic to be executed with a lower priority, and "slowly"
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author Maxime Bouton       <boutonm@stanford.edu>
 *  @author Jessie Lauzon       <jessicatlauzon@gmail.com>
 */


// include header file
#include "aa241x_low_control_law.h"
#include "aa241x_low_aux.h"

//#include "aa241x_low_custom_struct.h"

#include <uORB/uORB.h>

using namespace aa241x_low;

/*
*   My Variables
*/

// vector of waypoints
std::vector<wpt> wpt_vec;

// vector of directions
std::vector<Uvec> dir_vec;

bool has_passed = false; 

wpt start;
wpt current;
Uvec cur_dir;

int count = 0 ; // index of the waypoint
// # of waypoints
int num_wpt = 42 ; 
/*
*  My Functions
*/

// Initialize the vector of waypoints
void fill_wpt(){
	for (int i = 0; i<num_wpt;i++){
		wpt wpt1;
		wpt1.N = wpt_list[3*i+1];
		wpt1.E = wpt_list[3*i];
		wpt1.D = aal_parameters.des_alt;//wpt_list[3*i+2]+aal_parameters.des_alt ;
		wpt_vec.push_back(wpt1);
	}
}

// Initialize the vector of directions
void fill_dir(){
	for (int i=0; i<num_wpt-1;i++){
		Uvec uvec1;
		uvec1.uN = dir_list[2*i+1];
		uvec1.uE = dir_list[2*i] ;
		dir_vec.push_back(uvec1); 
	}
}


bool has_passed_wpt(wpt current, Uvec cur_dir){
	float prod = cur_dir.uE * (current.E - position_E) + cur_dir.uN * (current.N - position_N) ;
	bool has_passed = false ;
	if (prod < 0.0f ){                // check if it has crossed the line 
		has_passed = true ;
	}
	return has_passed ;
}



/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 *
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */
void low_loop()
{

	if (timestamp - previous_loop_timestamp >= 500000.0f){  // // Run if more than 0.5 seconds have passes since last loop, 

		fill_wpt();
		fill_dir();

		start   = wpt_vec[0];
		current = wpt_vec[1];
		cur_dir = dir_vec[0];

		low_data.uN = cur_dir.uN ;
		low_data.uE = cur_dir.uE ; 

		low_data.wN = current.N ;
		low_data.wE = current.E ;
		low_data.wD = aal_parameters.des_alt; 
		
		count = 0;


	}
	


	has_passed = has_passed_wpt(current, cur_dir) ; 
	
	if (has_passed == true){

		count+=1;

		current = wpt_vec[count+1];

		cur_dir = dir_vec[count];

		low_data.uN = cur_dir.uN ;
		low_data.uE = cur_dir.uE ; 

		low_data.wN = current.N ;
		low_data.wE = current.E ;
		//low_data.wD = current.D ;

	}

	if (count == num_wpt){

		count = 0 ;
		start   = wpt_vec[0];
		current = wpt_vec[1];
		cur_dir = dir_vec[0];
	}
	
}
