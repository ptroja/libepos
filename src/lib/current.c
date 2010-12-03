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

#include "current.h"

void epos_current_init(epos_current_p current, float target_value) {
  current->target_value = target_value;
}

int epos_current_setup(epos_node_p node, epos_current_config_p config) {
  int result;

  if (!(result = epos_current_set_p_gain(&node->dev, config->p_gain)))
    return epos_current_set_i_gain(&node->dev, config->i_gain);
  else
    return result;
}

int epos_current_start(epos_node_p node, epos_current_p current) {
  int result;
  short curr = current->target_value*1e3;

  if (!(result = epos_control_set_type(&node->control, 
      epos_control_current)) &&
    !(result = epos_control_start(&node->control)))
    result = epos_current_set_demand(&node->dev, curr);

  return result;
}

int epos_current_stop(epos_node_p node) {
  return epos_control_stop(&node->control);
}

short epos_current_get_actual(epos_device_p dev) {
  short current;
  epos_device_read(dev, EPOS_CURRENT_INDEX_ACTUAL_VALUE, 0,
    (unsigned char*)&current, sizeof(short));

  return current;
}

short epos_current_get_average(epos_device_p dev) {
  short current;
  epos_device_read(dev, EPOS_CURRENT_INDEX_AVERAGE_VALUE, 0,
    (unsigned char*)&current, sizeof(short));

  return current;
}

int epos_current_set_demand(epos_device_p dev, short current) {
  return epos_device_write(dev, EPOS_CURRENT_INDEX_SETTING_VALUE, 0,
    (unsigned char*)&current, sizeof(short));
}

short epos_current_get_demand(epos_device_p dev) {
  short current;
  epos_device_read(dev, EPOS_CURRENT_INDEX_SETTING_VALUE, 0,
    (unsigned char*)&current, sizeof(short));

  return current;
}

int epos_current_set_p_gain(epos_device_p dev, short p_gain) {
  return epos_device_write(dev, EPOS_CURRENT_INDEX_CONTROL_PARAMETERS,
    EPOS_CURRENT_SUBINDEX_P_GAIN, (unsigned char*)&p_gain, sizeof(short));
}

int epos_current_set_i_gain(epos_device_p dev, short i_gain) {
  return epos_device_write(dev, EPOS_CURRENT_INDEX_CONTROL_PARAMETERS,
    EPOS_CURRENT_SUBINDEX_I_GAIN, (unsigned char*)&i_gain, sizeof(short));
}
