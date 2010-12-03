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

#include "home.h"
#include "gear.h"

void epos_home_init(epos_home_p home, epos_home_method_t method, float current,
  float velocity, float acceleration, float position) {
  home->method = method;
  home->type = epos_profile_sinusoidal;

  home->current = current;
  home->switch_vel = velocity;
  home->zero_vel = velocity;
  home->acceleration = acceleration;

  home->offset = 0.0;
  home->position = position;
}

int epos_home_start(epos_node_p node, epos_home_p home) {
  int result;
  unsigned int switch_vel = abs(epos_gear_from_angular_velocity(&node->gear,
    home->switch_vel));
  unsigned int zero_vel = abs(epos_gear_from_angular_velocity(&node->gear,
    home->zero_vel));
  unsigned int acc = abs(epos_gear_from_angular_acceleration(&node->gear,
    home->acceleration));
  int offset = epos_gear_from_angle(&node->gear, home->offset);
  int pos = epos_gear_from_angle(&node->gear, home->position);

  if (!(result = epos_control_set_type(&node->control, epos_control_homing)) &&
    !(result = epos_home_set_method(&node->dev, home->method)) &&
    !(result = epos_home_set_current_threshold(&node->dev,
      home->current*1e3)) &&
    !(result = epos_home_set_switch_search_velocity(&node->dev, switch_vel)) &&
    !(result = epos_home_set_zero_search_velocity(&node->dev, zero_vel)) &&
    !(result = epos_home_set_acceleration(&node->dev, acc)) &&
    !(result = epos_home_set_offset(&node->dev, offset)) &&
    !(result = epos_home_set_position(&node->dev, pos)) &&
    !(result = epos_profile_set_type(&node->dev, home->type)) &&
    !(result = epos_device_set_control(&node->dev,
      EPOS_DEVICE_CONTROL_SHUTDOWN)) &&
    !(result = epos_control_start(&node->control)))
    result = epos_device_set_control(&node->dev, EPOS_HOME_CONTROL_START);

  epos_control_start(&node->control);
  epos_device_set_control(&node->dev, EPOS_HOME_CONTROL_START);

  return result;
}

int epos_home_stop(epos_node_p node) {
  int result;
  if (!(result = epos_device_set_control(&node->dev, EPOS_HOME_CONTROL_HALT)))
    return epos_control_stop(&node->control);
  else  
    return result;
}

int epos_home_wait(epos_node_p node, double timeout) {
  return epos_device_wait_status(&node->dev, EPOS_HOME_STATUS_REACHED,
    timeout);
}

int epos_home_set_method(epos_device_p dev, epos_home_method_t method) {
  unsigned char m = method;
  return epos_device_write(dev, EPOS_HOME_INDEX_METHOD, 0, &m, 1);
}

int epos_home_set_current_threshold(epos_device_p dev, short current) {
  return epos_device_write(dev, EPOS_HOME_INDEX_CURRENT_THRESHOLD, 0,
    (unsigned char*)&current, sizeof(short));
}

int epos_home_set_switch_search_velocity(epos_device_p dev, unsigned int
  velocity) {
  return epos_device_write(dev, EPOS_HOME_INDEX_VELOCITIES,
    EPOS_HOME_SUBINDEX_SWITCH_SEARCH_VELOCITY, (unsigned char*)&velocity,
    sizeof(unsigned int));
}

int epos_home_set_zero_search_velocity(epos_device_p dev, unsigned int
  velocity) {
  return epos_device_write(dev, EPOS_HOME_INDEX_VELOCITIES,
    EPOS_HOME_SUBINDEX_ZERO_SEARCH_VELOCITY, (unsigned char*)&velocity,
    sizeof(unsigned int));
}

int epos_home_set_acceleration(epos_device_p dev, unsigned int acceleration) {
  return epos_device_write(dev, EPOS_HOME_INDEX_ACCELERATION, 0,
    (unsigned char*)&acceleration, sizeof(unsigned int));
}

int epos_home_set_offset(epos_device_p dev, int offset) {
  return epos_device_write(dev, EPOS_HOME_INDEX_OFFSET, 0,
    (unsigned char*)&offset, sizeof(int));
}

int epos_home_set_position(epos_device_p dev, int position) {
  return epos_device_write(dev, EPOS_HOME_INDEX_POSITION, 0,
    (unsigned char*)&position, sizeof(int));
}
