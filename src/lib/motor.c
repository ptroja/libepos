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

#include "motor.h"

const char* epos_motor_errors[] = {
  "success",
  "error setting EPOS motor parameters",
};

void epos_motor_init(epos_motor_p motor, epos_device_p dev, epos_motor_type_t
  type, float max_current) {
  motor->dev = dev;

  motor->type = type;
  motor->max_cont_current = 0.5*max_current;
  motor->max_out_current = max_current;
  motor->num_poles = 1;
}

void epos_motor_destroy(epos_motor_p motor) {
  motor->dev = 0;
}

int epos_motor_setup(epos_motor_p motor) {
  if (!epos_motor_set_type(motor, motor->type) &&
    !epos_motor_set_max_continuous_current(motor,
      motor->max_cont_current*1e3) &&
    !epos_motor_set_max_output_current(motor, motor->max_out_current*1e3)) {
    return EPOS_MOTOR_ERROR_NONE;
  }
  else
    return EPOS_MOTOR_ERROR_SETUP;
}

epos_motor_type_t epos_motor_get_type(epos_motor_p motor) {
  short type;
  epos_device_read(motor->dev, EPOS_MOTOR_INDEX_TYPE, 0,
    (unsigned char*)&type, sizeof(short));

  return type;
}

int epos_motor_set_type(epos_motor_p motor, epos_motor_type_t type) {
  short t = type;
  int result = epos_device_write(motor->dev, EPOS_MOTOR_INDEX_TYPE, 0,
    (unsigned char*)&t, sizeof(short));

  if (!result)
    motor->type = type;

  return result;
}

short epos_motor_get_max_continuous_current(epos_motor_p motor) {
  short current;
  epos_device_read(motor->dev, EPOS_MOTOR_INDEX_DATA,
    EPOS_MOTOR_SUBINDEX_MAX_CONTINUOUS_CURRENT, (unsigned char*)&current,
    sizeof(short));

  return current;
}

int epos_motor_set_max_continuous_current(epos_motor_p motor, short current) {
  int result = epos_device_write(motor->dev, EPOS_MOTOR_INDEX_DATA,
    EPOS_MOTOR_SUBINDEX_MAX_CONTINUOUS_CURRENT, (unsigned char*)&current,
    sizeof(short));

  if (!result)
    motor->max_cont_current = current*1e-3;

  return result;
}

short epos_motor_get_max_output_current(epos_motor_p motor) {
  short current;
  epos_device_read(motor->dev, EPOS_MOTOR_INDEX_DATA,
    EPOS_MOTOR_SUBINDEX_MAX_OUTPUT_CURRENT, (unsigned char*)&current,
    sizeof(short));

  return current;
}

int epos_motor_set_max_output_current(epos_motor_p motor, short current) {
  int result = epos_device_write(motor->dev, EPOS_MOTOR_INDEX_DATA,
    EPOS_MOTOR_SUBINDEX_MAX_OUTPUT_CURRENT, (unsigned char*)&current,
    sizeof(short));

  if (!result)
    motor->max_out_current = current*1e-3;

  return result;
}

short epos_motor_get_num_poles(epos_motor_p motor) {
  short num_poles;
  epos_device_read(motor->dev, EPOS_MOTOR_INDEX_DATA,
    EPOS_MOTOR_SUBINDEX_NUM_POLES, (unsigned char*)&num_poles,
    sizeof(short));

  return num_poles;
}

int epos_motor_set_num_poles(epos_motor_p motor, short num_poles) {
  int result = epos_device_write(motor->dev, EPOS_MOTOR_INDEX_DATA,
    EPOS_MOTOR_SUBINDEX_NUM_POLES, (unsigned char*)&num_poles,
    sizeof(short));

  if (!result)
    motor->num_poles = num_poles;

  return result;
}
