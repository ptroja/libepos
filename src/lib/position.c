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
#include <float.h>

#include "position.h"
#include "gear.h"

void epos_position_init(epos_position_p position, float target_value) {
  epos_position_init_limits(position, target_value, -FLT_MAX, FLT_MAX,
    FLT_MAX);
}

void epos_position_init_limits(epos_position_p position, float target_value,
  float min_value, float max_value, float max_error) {
  position->target_value = target_value;

  position->min_value = min_value;
  position->max_value = max_value;
  position->max_error = max_error;
}

int epos_position_setup(epos_node_p node, epos_position_config_p config) {
  int result;

  if (!(result = epos_position_set_p_gain(&node->dev, config->p_gain)) &&
    !(result = epos_position_set_i_gain(&node->dev, config->i_gain)) &&
    !(result = epos_position_set_d_gain(&node->dev, config->d_gain)) &&
    !(result = epos_position_set_velocity_factor(&node->dev,
      config->vel_factor)))
    result = epos_position_set_acceleration_factor(&node->dev,
      config->acc_factor);

  return result;
}

int epos_position_start(epos_node_p node, epos_position_p position) {
  int result;
  int pos = epos_gear_from_angle(&node->gear, position->target_value);
  int min_pos = epos_gear_from_angle(&node->gear, position->min_value);
  int max_pos = epos_gear_from_angle(&node->gear, position->max_value);
  unsigned int max_error = abs(epos_gear_from_angle(&node->gear,
    position->max_error));

  if (!(result = epos_control_set_type(&node->control, 
      epos_control_position)) &&
    !(result = epos_position_set_limits(&node->dev, min_pos, max_pos)) &&
    !(result = epos_position_set_max_error(&node->dev, max_error)) &&
    !(result = epos_control_start(&node->control)))
    return epos_position_set_demand(&node->dev, pos);
  else
    return result;
}

int epos_position_stop(epos_node_p node) {
  return epos_control_stop(&node->control);
}

int epos_position_update(epos_node_p node, epos_position_p position) {
  int pos = epos_gear_from_angle(&node->gear, position->target_value);
  return epos_position_set_demand(&node->dev, pos);
}

int epos_position_set_limits(epos_device_p dev, int min_pos, int max_pos) {
  int result;

  if (!(result = epos_device_write(dev, EPOS_POSITION_INDEX_SOFTWARE_LIMIT,
    EPOS_POSITION_SUBINDEX_NEG_LIMIT, (unsigned char*)&min_pos, sizeof(int))))
    return epos_device_write(dev, EPOS_POSITION_INDEX_SOFTWARE_LIMIT,
    EPOS_POSITION_SUBINDEX_POS_LIMIT, (unsigned char*)&max_pos, sizeof(int));
  else
    return result;
}

int epos_position_set_max_error(epos_device_p dev, unsigned int max_error) {
  return epos_device_write(dev, EPOS_POSITION_INDEX_MAX_FOLLOWING_ERROR, 0,
    (unsigned char*)&max_error, sizeof(unsigned int));
}

int epos_position_get_actual(epos_device_p dev) {
  int pos;
  epos_device_read(dev, EPOS_POSITION_INDEX_ACTUAL_VALUE, 0,
    (unsigned char*)&pos, sizeof(int));

  return pos;
}

int epos_position_set_demand(epos_device_p dev, int position) {
  return epos_device_write(dev, EPOS_POSITION_INDEX_SETTING_VALUE, 0,
    (unsigned char*)&position, sizeof(int));
}

int epos_position_get_demand(epos_device_p dev) {
  int pos;
  epos_device_read(dev, EPOS_POSITION_INDEX_DEMAND_VALUE, 0,
    (unsigned char*)&pos, sizeof(int));

  return pos;
}

int epos_position_set_p_gain(epos_device_p dev, short p_gain) {
  return epos_device_write(dev, EPOS_POSITION_INDEX_CONTROL_PARAMETERS,
    EPOS_POSITION_SUBINDEX_P_GAIN, (unsigned char*)&p_gain, sizeof(short));
}

int epos_position_set_i_gain(epos_device_p dev, short i_gain) {
  return epos_device_write(dev, EPOS_POSITION_INDEX_CONTROL_PARAMETERS,
    EPOS_POSITION_SUBINDEX_I_GAIN, (unsigned char*)&i_gain, sizeof(short));
}

int epos_position_set_d_gain(epos_device_p dev, short d_gain) {
  return epos_device_write(dev, EPOS_POSITION_INDEX_CONTROL_PARAMETERS,
    EPOS_POSITION_SUBINDEX_D_GAIN, (unsigned char*)&d_gain, sizeof(short));
}

int epos_position_set_velocity_factor(epos_device_p dev, short vel_factor) {
  return epos_device_write(dev, EPOS_POSITION_INDEX_CONTROL_PARAMETERS,
    EPOS_POSITION_SUBINDEX_VELOCITY_FACTOR, (unsigned char*)&vel_factor,
    sizeof(short));
}

int epos_position_set_acceleration_factor(epos_device_p dev, short
  acc_factor) {
  return epos_device_write(dev, EPOS_POSITION_INDEX_CONTROL_PARAMETERS,
    EPOS_POSITION_SUBINDEX_ACCELERATION_FACTOR, (unsigned char*)&acc_factor,
    sizeof(short));
}
