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
 * @file aa241x_fw_control.cpp
 *
 * Secondary file to the fixedwing controller containing the
 * control law for the AA241x class.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author Maxime Bouton       <boutonm@stanford.edu>
 *  @author Jessie Lauzon       <jessicatlauzon@gmail.com>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)

int casenumber     = 0;

float des_alt      = low_data.wD; 
float des_speed    = aah_parameters.des_sp;
float des_yaw      = 0.0f;
float des_E_line   = low_data.uE;
float des_N_line   = low_data.uN;
float des_psi_line = atan2f(des_E_line,des_N_line) ;
float waypoint_E   = low_data.wE; 
float waypoint_N   = low_data.wN;


/**
 * Saturate Command
 */
float saturate_correction(float correction){
	if (correction > 1.0f) {
		 correction = 1.0f;
	} 
	else if (correction < -1.0f ) {
		 correction = -1.0f;
	}
	else{
	     correction = correction;
	}
	return correction;
}
//if (proportionalRollCorrection > 1.0f) {
//	proportionalRollCorrection = 1.0f;
//} else if (proportionalRollCorrection < -1.0f ) {
//	proportionalRollCorrection = -1.0f;
//}

/**
* Speed Control
*/
void speed_control(float des_speed, float throttle_trim){
	high_data.delta_u = aah_parameters.K_u * ( des_speed - speed_body_u) + aah_parameters.throttle_trim + throttle_trim;
	if (high_data.delta_u < 0.0f){
		high_data.delta_u = 0.0;	
	}
	else if (high_data.delta_u > 1.0f){
		high_data.delta_u = 1.0;
	}
	throttle_servo_out = high_data.delta_u ; 
}

/** 
* Altitude Control
*/
void altitude_control(float des_alt){
	
	float des_pitch = aah_parameters.K_h * (des_alt - (-position_D_gps));
	if (des_pitch > 0.6f){
		des_pitch = 0.6f ;
	}
	else if (des_pitch < -0.6f){
		des_pitch = -0.6f;
	}
	high_data.delta_e = aah_parameters.K_th * des_pitch - aah_parameters.K_th * pitch  + aah_parameters.elevator_trim;
	high_data.delta_e = saturate_correction(high_data.delta_e);
	pitch_servo_out = high_data.delta_e ;
}

/** 
* Heading Control using roll
*/
void heading_control(float des_yaw){
	float delta_yaw = (des_yaw - yaw);
	if (delta_yaw < -3.14f){
		delta_yaw += 6.28f ;
	}
	else if (delta_yaw > 3.14f){
		delta_yaw -= 6.28f ;
	}

	float roll_command = aah_parameters.K_psi * delta_yaw;
	
	if (roll_command>1.34f){
		roll_command = 1.34f ;
	}
	else if (roll_command <-1.34f){
		roll_command = -1.34f ;
	}

	high_data.delta_a = aah_parameters.K_phi * (roll_command - roll) + aah_parameters.ailerons_trim;
	high_data.delta_a = saturate_correction(high_data.delta_a);
	roll_servo_out = high_data.delta_a;
}

/**
* Yaw Control 
*/

void yaw_control(float des_yaw){
	high_data.delta_r = aah_parameters.K_rud * (- yaw_rate) + aah_parameters.rudder_trim; //  (0-yaw_rate)
	high_data.delta_r = saturate_correction(high_data.delta_r);
	yaw_servo_out = high_data.delta_r;
}

/**
* Line Tracking
*/
float line_tracking(float des_E_line, float des_N_line, float des_psi_line, float waypoint_E, float waypoint_N){
	float y  = des_N_line * (waypoint_E - position_E) - des_E_line * (waypoint_N - position_N);
	float yc = 0.0f;
	float cor = aah_parameters.K_d * (yc-y) ;
	if (cor > 1.22f){
		cor = 1.22f ;
	}
	else if (cor < -1.22f){
		cor = -1.22f ;
	}
	des_yaw = cor + des_psi_line ;
	if (des_yaw < -3.14f){
		des_yaw += 6.28f ;
	}
	else if (des_yaw > 3.14f){
		des_yaw -= 6.28f ;
	}
	return des_yaw ;
}


/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */
void flight_control() {

//	float my_float_variable = 0.0f;		/**< example float variable */


	// An example of how to run a one time 'setup' for example to lock one's altitude and heading...
	if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop, 
																	 //	should only occur on first engagement since this is 59Hz loop
		des_yaw            = yaw; 							                 
		des_alt            = low_data.wD ;             		         // altitude_desired needs to be declared
		des_speed          = aah_parameters.des_sp ;
		high_data.throttle_trim = man_throttle_in ;



	}

	switch(aah_parameters.case_no){

		// FULL AUTO
	case 0 : 
		{
			des_alt      = low_data.wD; 
			speed_control(des_speed, high_data.throttle_trim);
			altitude_control(des_alt);
			heading_control(des_yaw);
			yaw_control(des_yaw);
			break;
		}

		// LATERAL ONLY
	case 1 :
		{
			heading_control(des_yaw);
			yaw_control(des_yaw);
			pitch_servo_out    = -man_pitch_in;
			throttle_servo_out = man_throttle_in;	 
			break;
		}
		// YAW ONLY
	case 11 :
		{
			yaw_control(des_yaw);
			roll_servo_out     = man_roll_in;
			pitch_servo_out    = -man_pitch_in;
			throttle_servo_out = man_throttle_in;
			break;
		}

		// LONGITUDINAL ONLY
	case 2 :
		{
			des_alt      = low_data.wD; 
			speed_control(des_speed,high_data.throttle_trim);
			altitude_control(des_alt);
			roll_servo_out = man_roll_in;
			yaw_servo_out  = man_yaw_in;
			break;
		}

		// SPEED ONLY 
	case 22 :
		{
			speed_control(des_speed,high_data.throttle_trim);
			pitch_servo_out = -man_pitch_in; 
			roll_servo_out  = man_roll_in;
			yaw_servo_out   = man_yaw_in;
			break;
		}

		// ALTITUDE ONLY
	case 23 :
		{
			des_alt      = low_data.wD; 
			altitude_control(des_alt);
			throttle_servo_out = man_throttle_in;
			roll_servo_out     = man_roll_in;
			yaw_servo_out      = man_yaw_in;
			break;
		}


		// LINE TRACKING
	case 10 : 
		{
		   des_E_line   = low_data.uE;
		   des_N_line   = low_data.uN;
	       des_psi_line = atan2f(des_E_line,des_N_line) ;
		   waypoint_E   = low_data.wE; 
		   waypoint_N   = low_data.wN;
		   des_yaw      = line_tracking(des_E_line, des_N_line, des_psi_line, waypoint_E, waypoint_N);
		   des_alt      = low_data.wD; 

		   speed_control(des_speed, high_data.throttle_trim);
		   altitude_control(des_alt);
		   heading_control(des_yaw);
		   yaw_control(des_yaw);
		   break;
		}

		// DEFAULT IS FULL MANUAL 
	default:
		{

            roll_servo_out     = man_roll_in;
            pitch_servo_out    = -man_pitch_in;
            yaw_servo_out      = man_yaw_in;
            throttle_servo_out = man_throttle_in;
			break;
		}


	}

	// TODO: write all of your flight control here...


	// getting low data value example
	// float my_low_data = low_data.variable_name1;

	// setting high data value example
	//high_data.variable_name1 = my_float_variable;


	// // Make a really simple proportional roll stabilizer // //
	//
	
	//float roll_desired = 0.0f; // roll_desired already exists in aa241x_high_aux so no need to repeat float declaration

	//// Now use your parameter gain and multiply by the error from desired
	//float proportionalRollCorrection = aah_parameters.proportional_roll_gain * (roll - roll_desired);

	//// Note the use of x.0f, this is important to specify that these are single and not double float values!

	//// Do bounds checking to keep the roll correction within the -1..1 limits of the servo output
	//if (proportionalRollCorrection > 1.0f) {
	//	proportionalRollCorrection = 1.0f;
	//} else if (proportionalRollCorrection < -1.0f ) {
	//	proportionalRollCorrection = -1.0f;
	//}

	// ENSURE THAT YOU SET THE SERVO OUTPUTS!!!
	// outputs should be set to values between -1..1 (except throttle is 0..1)
	// where zero is no actuation, and -1,1 are full throw in either the + or - directions

	// Set output of roll servo to the control law output calculated above
	//roll_servo_out = proportionalRollCorrection;		
	// as an example, just passing through manual control to everything but roll
	//pitch_servo_out = -man_pitch_in;
	//yaw_servo_out = man_yaw_in;
	//throttle_servo_out = man_throttle_in;
	//casenumber = aah_parameters.case_no;
	//if ((casenumber = 1) || (casenumber = 3)) {
	////  // body velocity u control // //
	//// Compute Correction
	//float proportionalBodyUCorrection = aah_parameters.proportional_velocity_gain * (speed_body_u - speed_body_u_desired);

	//// Do bounds checking to keep the velocity correction within the -1..1 limits of the servo output
	////saturate_correction(proportionalBodyUCorrection)
	//if (proportionalBodyUCorrection > 1.0f) {
	//	proportionalBodyUCorrection = 1.0f;
	//} else if (proportionalBodyUCorrection < -1.0f ) {
	//	proportionalBodyUCorrection = -1.0f;
	//}

	//// Set output of throttle servo to the control law output calculated above
	//throttle_servo_out = proportionalBodyUCorrection;	

	//// //  Elevator output from closed loop on altitude and pitch

	//// Compute Correction
	//float deltaECorrection = aah_parameters.proportional_pitch_gain * aah_parameters.proportional_altitude_gain * 
	//	(altitude_desired - (-position_D_gps))-aah_parameters.proportional_pitch_gain * pitch;
	//
	//// Do bounds checking to keep the elevator input correction within the -1..1 limits of the servo output
	//if (deltaECorrection > 1.0f) {
	//	deltaECorrection = 1.0f;
	//} else if (deltaECorrection < -1.0f ) {
	//	deltaECorrection = -1.0f;
	//}
	//
	//// Set output of elevator servo 
	//pitch_servo_out = deltaECorrection ;
	//}
	//if ((casenumber = 2) || (casenumber = 3)) {
	//// // YAW CONTROL // //
	//// Compute Correction
	//float deltaRudder = aah_parameters.proportional_yaw_gain * (yaw_desired - yaw ) ;
	//	

	//// Do bounds checking to keep the elevator input correction within the -1..1 limits of the servo output
	//if (deltaRudder > 1.0f) {
	//	deltaRudder = 1.0f;
	//} else if (deltaRudder < -1.0f ) {
	//	deltaRudder = -1.0f;
	//}

	//yaw_servo_out = man_yaw_in; 

	//// // ROLL CONTROL // //
	//// Compute correction
	//float roll_desired = aah_parameters.proportional_phi_command*(yaw_desired-yaw);
	//float deltaAilerons = aah_parameters.proportional_phi_gain * (roll_desired - roll);
	//	

	//// Bounds checking

	//// Do bounds checking to keep the elevator input correction within the -1..1 limits of the servo output
	//if (deltaAilerons > 1.0f) {
	//	deltaAilerons = 1.0f;
	//} else if (deltaAilerons < -1.0f ) {
	//	deltaAilerons = -1.0f;
	//}

	//// Set output of ailerons servo
	//roll_servo_out = deltaAilerons ; 

	//}

}
