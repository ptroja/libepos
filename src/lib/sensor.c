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

#include "sensor.h"

const char* epos_sensor_errors[] = {
  "success",
  "error setting EPOS sensor parameters",
};

void epos_sensor_init(epos_sensor_p sensor, epos_device_p dev,
  epos_sensor_type_t type, epos_sensor_polarity_t polarity, int num_pulses,
  epos_sensor_supervision_t supervision) {
  sensor->dev = dev;

  sensor->type = type;
  sensor->polarity = polarity;
  sensor->num_pulses = num_pulses;

  sensor->supervision = supervision;
}

void epos_sensor_destroy(epos_sensor_p sensor) {
  sensor->dev = 0;
}

int epos_sensor_setup(epos_sensor_p sensor) {
  if (!epos_sensor_set_type(sensor, sensor->type) &&
    !epos_sensor_set_polarity(sensor, sensor->polarity) &&
    !epos_sensor_set_pulses(sensor, sensor->num_pulses) &&
    !epos_sensor_set_supervision(sensor, sensor->supervision))
    return EPOS_SENSOR_ERROR_NONE;
  else
    return EPOS_SENSOR_ERROR_SETUP;
}

epos_sensor_type_t epos_sensor_get_type(epos_sensor_p sensor) {
  short type;
  epos_device_read(sensor->dev, EPOS_SENSOR_INDEX_CONFIGURATION,
    EPOS_SENSOR_SUBINDEX_TYPE, (unsigned char*)&type, sizeof(short));

  return type;
}

int epos_sensor_set_type(epos_sensor_p sensor, epos_sensor_type_t type) {
  short t = type;
  int result = epos_device_write(sensor->dev, EPOS_SENSOR_INDEX_CONFIGURATION,
    EPOS_SENSOR_SUBINDEX_TYPE, (unsigned char*)&t, sizeof(short));

  if (!result)
    sensor->type = type;

  return result;
}

epos_sensor_polarity_t epos_sensor_get_polarity(epos_sensor_p sensor) {
  short polarity;
  epos_device_read(sensor->dev, EPOS_SENSOR_INDEX_CONFIGURATION,
    EPOS_SENSOR_SUBINDEX_POLARITY, (unsigned char*)&polarity, sizeof(short));

  return polarity;
}

int epos_sensor_set_polarity(epos_sensor_p sensor, epos_sensor_polarity_t
  polarity) {
  short p = polarity;
  int result = epos_device_write(sensor->dev, EPOS_SENSOR_INDEX_CONFIGURATION,
    EPOS_SENSOR_SUBINDEX_POLARITY, (unsigned char*)&p, sizeof(short));

  if (!result)
    sensor->polarity = polarity;

  return result;
}

int epos_sensor_get_pulses(epos_sensor_p sensor) {
  int pulses;

  if (sensor->dev->hardware_generation == 1) {
    short pulses_short;
    epos_device_read(sensor->dev, EPOS_SENSOR_INDEX_CONFIGURATION,
      EPOS_SENSOR_SUBINDEX_PULSES, (unsigned char*)&pulses_short,
      sizeof(short));
    pulses = pulses_short;
  }
  else
    epos_device_read(sensor->dev, EPOS_SENSOR_INDEX_CONFIGURATION,
    EPOS_SENSOR_SUBINDEX_PULSES, (unsigned char*)&pulses, sizeof(int));

  return pulses;
}

int epos_sensor_set_pulses(epos_sensor_p sensor, int num_pulses) {
  int result;

  if (sensor->dev->hardware_generation == 1) {
    short pulses_short = num_pulses;;    
    result = epos_device_write(sensor->dev, EPOS_SENSOR_INDEX_CONFIGURATION,
      EPOS_SENSOR_SUBINDEX_PULSES, (unsigned char*)&pulses_short,
      sizeof(short));
  }
  else
    result = epos_device_write(sensor->dev, EPOS_SENSOR_INDEX_CONFIGURATION,
    EPOS_SENSOR_SUBINDEX_PULSES, (unsigned char*)&num_pulses, sizeof(int));

  if (!result)
    sensor->num_pulses = num_pulses;

  return result;
}

epos_sensor_supervision_t epos_sensor_get_supervision(epos_sensor_p sensor) {
  return epos_device_get_configuration(sensor->dev) & 0x0003;
}

int epos_sensor_set_supervision(epos_sensor_p sensor, epos_sensor_supervision_t
  supervision) {
  short configuration = epos_device_get_configuration(sensor->dev);
  configuration = supervision | (0xFFFC & configuration);

  int result = epos_device_set_configuration(sensor->dev, configuration);

  if (!result)
    sensor->supervision = supervision;

  return result;
}

short epos_sensor_get_position(epos_sensor_p sensor) {
  short pos;
  epos_device_read(sensor->dev, EPOS_SENSOR_INDEX_POSITION, 0,
    (unsigned char*)&pos, sizeof(short));

  return pos;
}
