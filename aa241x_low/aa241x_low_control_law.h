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
 * @file aa241x_low.h
 *
 * Header file for student's low priority control law.
 * Runs at ~ XX HZ TODO: figure out XX
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author Maxime Bouton       <boutonm@stanford.edu>
 *  @author Jessie Lauzon       <jessicatlauzon@gmail.com>
 */
#ifndef AA241X_LOW_H_
#define AA241X_LOW_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

/*
 * Declare variables here that you may want to access
 * in multiple different function.
 */


// Waypoint Structure
// (N,E,D) in Lake lag coordinate system
typedef struct wpt wpt;
struct wpt
{
	float N;
	float E;
	float D; // Assume vector pointing down (negative altitudes)

	wpt()
	{
		N = 0.0;
		E = 0.0;
		D = 40.0;
	}
	wpt(float N0, float E0, float D0)
	{
		N = N0 ;
		E = E0;
		D = D0 ;
	}
};


// Line vector structure
// (N,E) normalized coordinates
typedef struct  Uvec Uvec;
struct Uvec
{
	float uN;
	float uE;

	Uvec()
	{
		uN = 0.0;
		uE = 1.0;
	}
	Uvec(float uN0, float uE0)
	{
		uN = uN0;
		uE = uE0;
	}
};


/*
 * Declare function prototypes here.
 */
bool has_passed_wpt(wpt current, Uvec cur_dir);

void fill_wpt();

void fill_dir();


/*
*  My list of waypoints
*/


float wpt_list[] = {
	-88,155.6017,0,						
	129.2243,56.06392709,0,				
	134.3962,50.42209002,0.3125,		
	134.0639,42.77563902,0.625,			
	128.4220,37.6037614,0.9375,			
	120.7756,37.93607291,1.25,			
	115.6037,43.57790998,1.5625,		
	115.9360,51.22436098,1.875,			
	121.5779,56.3962386,2.1875,			
	129.2243,56.06392709,2.5,			
	134.3962,50.42209002,2.8125,		
	134.0639,42.77563902,3.125,			
	128.42209,37.6037614,3.4375,		
	120.775639,37.93607291,3.75,		
	115.6037614,43.57790998,4.0625,		
	115.9360729,51.22436098,4.375,		
	121.57791,56.3962386,4.6875,		
	129.224361,56.06392709,5,			
	134.3962386,50.42209002,5.3125,		
	134.0639271,42.77563902,5.625,		
	130.7374106,38.80963253,5.625,		
	-69.26537359,-101.1923171,-1,		
	-76.73784977,-102.8478362,-0.6875,
	-83.19231713,-98.73462641,-0.375,
	-84.84783622,-91.26215023,-0.0625,
	-80.73462641,-84.80768287,0.25,		
	-73.26215023,-83.15216378,0.5625,
	-66.80768287,-87.26537359,0.875,	
	-65.15216378,-94.73784977,1.1875,
	-69.26537359,-101.1923171,1.5,		
	-76.73784977,-102.8478362,1.8125,
	-83.19231713,-98.73462641,2.125,	
	-84.84783622,-91.26215023,2.4375,
	-80.73462641,-84.80768287,2.75,		
	-73.26215023,-83.15216378,3.0625,
	-66.80768287,-87.26537359,3.375,	
	-65.15216378,-94.73784977,3.6875,
	-69.26537359,-101.1923171,4,		
	-76.73784977,-102.8478362,4.3125,
	-83.19231713,-98.73462641,4.625,	
	-84.96206795,-93.87017359,4.625,	
	-104,		 144.3983,	  -1
	
};

float dir_list[] = {
	0.9091019,-0.416573805,
	0.675738377,-0.73714153,
	-0.043418592,-0.99905696,
	-0.737141538,-0.675738377,
	-0.999056968,0.043418592,
	-0.675738377,0.737141538,
	0.043418592,0.999056968,
	0.737141538,0.675738377,
	0.999056968,-0.043418592,
	0.675738377,-0.737141538,
	-0.043418592,-0.999056968,
	-0.737141538,-0.675738377,
	-0.999056968,0.043418592,
	-0.675738377,0.737141538,
	0.043418592,0.999056968,
	0.737141538,0.675738377,
	0.999056968,-0.043418592,
	0.675738377,-0.737141538,
	-0.043418592,-0.999056968,
	-0.642633632,-0.766173619,
	-0.819231919,-0.573462346,
	-0.976326064,-0.216303993,
	-0.843316801,0.53741676,
	-0.216303993,0.976326064,
	0.53741676,0.843316801,
	0.976326064,0.216303993,
	0.843316801,-0.53741676,
	0.216303993,-0.976326064,
	-0.53741676,-0.843316801,
	-0.976326064,-0.216303993,
	-0.843316801,0.53741676,
	-0.216303993,0.976326064,
	0.53741676,0.843316801,
	0.976326064,0.216303993,
	0.843316801,-0.53741676,
	0.216303993,-0.976326064,
	-0.53741676,-0.843316801,
	-0.976326064,-0.216303993,
	-0.843316801,0.53741676,
	-0.341889606,0.939740122,
	-0.079647342,0.996823104
};


#endif /* AA241X_SLOW_H_ */
