/*
Copyright 2008 Ralf Kaestner <ralf.kaestner@gmail.com>
ASL-ETHZ http://www.asl.ethz.ch/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <float.h>

#include "gear.h"

void epos_gear_init(epos_gear_p gear, epos_sensor_p sensor, float
  transmission) {
  gear->sensor = sensor;
  gear->transmission = transmission;
}

void epos_gear_destroy(epos_gear_p gear) {
  gear->sensor = 0;
}

float epos_gear_to_angle(epos_gear_p gear, int position) {
  return 2.0*M_PI*position/(4.0*gear->sensor->num_pulses*gear->transmission);
}

int epos_gear_from_angle(epos_gear_p gear, float angle) {
  return clip(angle*4.0*gear->sensor->num_pulses*gear->transmission/(2.0*M_PI),
    INT_MIN, INT_MAX);
}

float epos_gear_to_angular_velocity(epos_gear_p gear, int velocity) {
  return 2.0*M_PI*velocity/(60.0*gear->transmission);
}

int epos_gear_from_angular_velocity(epos_gear_p gear, float angular_vel) {
  return clip(angular_vel*60.0*gear->transmission/(2.0*M_PI),
    INT_MIN, INT_MAX);
}

float epos_gear_to_angular_acceleration(epos_gear_p gear, int acceleration) {
  return 2.0*M_PI*acceleration/(60.0*gear->transmission);
}

int epos_gear_from_angular_acceleration(epos_gear_p gear, float angular_acc) {
  return clip(angular_acc*60.0*abs(gear->transmission)/(2.0*M_PI), 
    INT_MIN, INT_MAX);
}
