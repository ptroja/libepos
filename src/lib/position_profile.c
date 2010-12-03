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

#include <tulibs/timer.h>

#include "position_profile.h"
#include "gear.h"

void epos_position_profile_init(epos_position_profile_p profile,
  float target_value, float velocity, float acceleration, float deceleration,
  epos_profile_type_t type) {
  profile->target_value = target_value;
  profile->velocity = velocity;
  profile->acceleration = acceleration;
  profile->deceleration = deceleration;

  profile->relative = 0;
  profile->type = type;

  profile->start_value = 0.0;
  profile->start_time = 0.0;
}

int epos_position_profile_start(epos_node_p node, epos_position_profile_p
  profile) {
  int result;
  int pos = epos_gear_from_angle(&node->gear, profile->target_value);
  unsigned int vel = abs(epos_gear_from_angular_velocity(&node->gear,
    profile->velocity));
  unsigned int acc = abs(epos_gear_from_angular_acceleration(&node->gear,
    profile->acceleration));
  unsigned int dec = abs(epos_gear_from_angular_acceleration(&node->gear,
    profile->deceleration));
  short control = (profile->relative) ?
    EPOS_POSITION_PROFILE_CONTROL_SET_RELATIVE :
    EPOS_POSITION_PROFILE_CONTROL_SET_ABSOLUTE;
  double t_min, t_max;

  if (!(result = epos_control_set_type(&node->control, 
      epos_control_profile_pos)) &&
    !(result = epos_position_profile_set_velocity(&node->dev, vel)) &&
    !(result = epos_profile_set_acceleration(&node->dev, acc)) &&
    !(result = epos_profile_set_deceleration(&node->dev, dec)) &&
    !(result = epos_profile_set_type(&node->dev, profile->type)) &&
    !(result = epos_control_start(&node->control)) &&
    !(result = epos_position_profile_set_target(&node->dev, pos))) {
    profile->start_value = epos_get_position(node);
    timer_start(&profile->start_time);
    result = epos_device_set_control(&node->dev, control);
    timer_correct(&profile->start_time);
  }

  return result;
}

int epos_position_profile_stop(epos_node_p node) {
  return epos_control_stop(&node->control);
}

float epos_position_profile_estimate(epos_position_profile_p profile,
  double time) {
  float s_0 = profile->start_value;
  float s_1 = (profile->relative) ? s_0+profile->target_value :
    profile->target_value;
  float d_s = 0.0;
  double t = time-profile->start_time;

  if (t > 0.0) {
    float s = fabs(s_1-s_0);
    float v = profile->velocity;
    float a = profile->acceleration;
    float d = profile->deceleration;

    if (profile->type == epos_profile_sinusoidal) {
      float v_max = sqrt(4.0*s/(M_PI/a+M_PI/d));
      v = min(v, v_max);

      double t_a = 0.5*M_PI*v/a;
      double t_d = 0.5*M_PI*v/d;
      float s_a = 0.25*sqr(v)*M_PI/a;
      float s_d = 0.25*sqr(v)*M_PI/d;
      double t_c = (s-s_a-s_d > 0.0) ? (s-s_a-s_d)/v : 0.0;
      float s_c = v*t_c;

      if (t < t_a)
        d_s = 0.5*v*(t-0.5*v/a*sin(2.0*a/v*t));
      else if (t < t_a+t_c)
        d_s = s_a+v*(t-t_a);
      else if (t < t_a+t_c+t_d)
        d_s = s_a+s_c+0.5*v*(t-t_a-t_c+0.5*v/d*sin(2.0*d/v*(t-t_a-t_c)));
      else
        d_s = s;
    }
    else {
      float v_max = sqrt(2.0*s/(1.0/a+1.0/d));
      v = min(v, v_max);

      double t_a = v/a;
      double t_d = v/d;
      float s_a = 0.5*sqr(v)/a;
      float s_d = 0.5*sqr(v)/d;
      double t_c = (s-s_a-s_d > 0.0) ? (s-s_a-s_d)/v : 0.0;
      float s_c = v*t_c;

      if (t < t_a)
        d_s = 0.5*a*sqr(t);
      else if (t < t_a+t_c)
        d_s = s_a+v*(t-t_a);
      else if (t < t_a+t_c+t_d)
        d_s = s_a+s_c+v*(t-t_a-t_c)-0.5*d*sqr(t-t_a-t_c);
      else
        d_s = s;
    }
  }

  if (s_1 > s_0)
    return s_0+d_s;
  else
    return s_0-d_s;
}

int epos_position_profile_set_target(epos_device_p dev, int position) {
  return epos_device_write(dev, EPOS_POSITION_PROFILE_INDEX_TARGET, 0,
    (unsigned char*)&position, sizeof(int));
}

int epos_position_profile_set_velocity(epos_device_p dev, unsigned int
  velocity) {
  return epos_device_write(dev, EPOS_POSITION_PROFILE_INDEX_VELOCITY, 0,
    (unsigned char*)&velocity, sizeof(unsigned int));
}
