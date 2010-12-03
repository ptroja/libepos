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

#include "velocity.h"
#include "gear.h"

void epos_velocity_init(epos_velocity_p velocity, float target_value) {
  velocity->target_value = target_value;
}

int epos_velocity_setup(epos_node_p node, epos_velocity_config_p config) {
  int result;

  if (!(result = epos_velocity_set_p_gain(&node->dev, config->p_gain)))
    return epos_velocity_set_i_gain(&node->dev, config->i_gain);
  else
    return result;
}

int epos_velocity_start(epos_node_p node, epos_velocity_p velocity) {
  int result;
  int vel = epos_gear_from_angular_velocity(&node->gear,
    velocity->target_value);

  if (!(result = epos_control_set_type(&node->control, 
      epos_control_velocity)) &&
    !(result = epos_control_start(&node->control)))
    result = epos_velocity_set_demand(&node->dev, vel);

  return result;
}

int epos_velocity_stop(epos_node_p node) {
  return epos_control_stop(&node->control);
}

int epos_velocity_update(epos_node_p node, epos_velocity_p velocity) {
  int vel = epos_gear_from_angular_velocity(&node->gear,
    velocity->target_value);

  return epos_velocity_set_demand(&node->dev, vel);
}

int epos_velocity_get_actual(epos_device_p dev) {
  int vel;
  epos_device_read(dev, EPOS_VELOCITY_INDEX_ACTUAL_VALUE, 0,
    (unsigned char*)&vel, sizeof(int));

  return vel;
}

int epos_velocity_get_average(epos_device_p dev) {
  int vel;
  epos_device_read(dev, EPOS_VELOCITY_INDEX_AVERAGE_VALUE, 0,
    (unsigned char*)&vel, sizeof(int));

  return vel;
}

int epos_velocity_set_demand(epos_device_p dev, int velocity) {
  return epos_device_write(dev, EPOS_VELOCITY_INDEX_SETTING_VALUE, 0,
    (unsigned char*)&velocity, sizeof(int));
}

int epos_velocity_get_demand(epos_device_p dev) {
  int vel;
  epos_device_read(dev, EPOS_VELOCITY_INDEX_DEMAND_VALUE, 0,
    (unsigned char*)&vel, sizeof(int));

  return vel;
}

int epos_velocity_set_p_gain(epos_device_p dev, short p_gain) {
  return epos_device_write(dev, EPOS_VELOCITY_INDEX_CONTROL_PARAMETERS,
    EPOS_VELOCITY_SUBINDEX_P_GAIN, (unsigned char*)&p_gain, sizeof(short));
}

int epos_velocity_set_i_gain(epos_device_p dev, short i_gain) {
  return epos_device_write(dev, EPOS_VELOCITY_INDEX_CONTROL_PARAMETERS,
    EPOS_VELOCITY_SUBINDEX_I_GAIN, (unsigned char*)&i_gain, sizeof(short));
}
