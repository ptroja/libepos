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

#include "velocity_profile.h"
#include "gear.h"

void epos_velocity_profile_init(epos_velocity_profile_p profile,
  float target_value, float acceleration, float deceleration,
  epos_profile_type_t type) {
  profile->target_value = target_value;
  profile->acceleration = acceleration;
  profile->deceleration = deceleration;

  profile->type = type;
}

int epos_velocity_profile_start(epos_node_p node, epos_velocity_profile_p
  profile) {
  int result;
  int vel = epos_gear_from_angular_velocity(&node->gear,
    profile->target_value);
  unsigned int acc = abs(epos_gear_from_angular_acceleration(&node->gear,
    profile->acceleration));
  unsigned int dec = abs(epos_gear_from_angular_acceleration(&node->gear,
    profile->deceleration));

  if (!(result = epos_control_set_type(&node->control, 
      epos_control_profile_vel)) &&
    !(result = epos_profile_set_acceleration(&node->dev, acc)) &&
    !(result = epos_profile_set_deceleration(&node->dev, dec)) &&
    !(result = epos_profile_set_type(&node->dev, profile->type)) &&
    !(result = epos_control_start(&node->control)))
    result = epos_velocity_profile_set_target(&node->dev, vel);

  return result;
}

int epos_velocity_profile_stop(epos_node_p node) {
  return epos_control_stop(&node->control);
}

int epos_velocity_profile_set_target(epos_device_p dev, int velocity) {
  return epos_device_write(dev, EPOS_VELOCITY_PROFILE_INDEX_TARGET, 0,
    (unsigned char*)&velocity, sizeof(int));
}
