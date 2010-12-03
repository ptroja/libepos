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
#include <string.h>

#include "epos.h"

#include "global.h"
#include "home.h"
#include "position.h"
#include "velocity.h"
#include "current.h"

const char* epos_errors[] = {
  "success",
  "error opening EPOS",
  "error closing EPOS",
  "error homing EPOS",
};

param_t epos_default_params[] = {
  {EPOS_PARAMETER_ID, "0"},
  {EPOS_PARAMETER_RESET, "1"},
  {EPOS_PARAMETER_SENSOR_TYPE, "1"},
  {EPOS_PARAMETER_SENSOR_POLARITY, "0"},
  {EPOS_PARAMETER_SENSOR_PULSES, "1024"},
  {EPOS_PARAMETER_SENSOR_SUPERVISION, "0"},
  {EPOS_PARAMETER_MOTOR_TYPE, "1"},
  {EPOS_PARAMETER_MOTOR_CURRENT, "2.0"},
  {EPOS_PARAMETER_GEAR_TRANSMISSION, "1.0"},
  {EPOS_PARAMETER_CONTROL_TYPE, "6"},

  {EPOS_PARAMETER_HOME_METHOD, "-1"},
  {EPOS_PARAMETER_HOME_TYPE, "0"},
  {EPOS_PARAMETER_HOME_CURRENT, "1.0"},
  {EPOS_PARAMETER_HOME_VELOCITY, "10.0"},
  {EPOS_PARAMETER_HOME_ACCELERATION, "10.0"},
  {EPOS_PARAMETER_HOME_OFFSET, "0.0"},
  {EPOS_PARAMETER_HOME_POSITION, "0.0"},
};

config_t epos_default_config = {
  epos_default_params,
  sizeof(epos_default_params)/sizeof(param_t),
};

void epos_init(epos_node_p node, can_device_p can_dev, config_p config) {
  if (!can_dev) {
    can_dev = malloc(sizeof(can_device_t));
    can_init(can_dev, 0);
  }

  config_init_default(&node->config, &epos_default_config);
  if (config)
    config_set(&node->config, config);

  epos_device_init(&node->dev, can_dev,
    config_get_int(&node->config, EPOS_PARAMETER_ID),
    config_get_int(&node->config, EPOS_PARAMETER_RESET));
  epos_sensor_init(&node->sensor, &node->dev,
    config_get_int(&node->config, EPOS_PARAMETER_SENSOR_TYPE),
    config_get_int(&node->config, EPOS_PARAMETER_SENSOR_POLARITY),
    config_get_int(&node->config, EPOS_PARAMETER_SENSOR_PULSES),
    config_get_int(&node->config, EPOS_PARAMETER_SENSOR_SUPERVISION));
  epos_motor_init(&node->motor, &node->dev,
    config_get_int(&node->config, EPOS_PARAMETER_MOTOR_TYPE),
    config_get_float(&node->config, EPOS_PARAMETER_MOTOR_CURRENT));
  epos_gear_init(&node->gear, &node->sensor,
    config_get_float(&node->config, EPOS_PARAMETER_GEAR_TRANSMISSION));
  epos_input_init(&node->input, &node->dev);
  epos_control_init(&node->control, &node->dev,
    config_get_int(&node->config, EPOS_PARAMETER_CONTROL_TYPE));
}

void epos_init_arg(epos_node_p node, int argc, char **argv, const char* 
  prefix) {
  can_device_p can_dev = malloc(sizeof(can_device_t));
  can_init_arg(can_dev, argc, argv, 0);

  config_t config;
  if (config_init_arg(&config, argc, argv, (prefix) ? prefix : 
      EPOS_CONFIG_ARG_PREFIX)) {
    config_print_help(stdout, &epos_default_config, EPOS_CONFIG_ARG_PREFIX);
    config_print_help(stdout, &can_default_config, CAN_CONFIG_ARG_PREFIX);
    exit(0);
  }
    
  epos_init(node, can_dev, &config);

  config_destroy(&config);
}

void epos_destroy(epos_node_p node) {
  can_device_p can_dev = node->dev.can_dev;

  epos_control_destroy(&node->control);
  epos_gear_destroy(&node->gear);
  epos_motor_destroy(&node->motor);
  epos_sensor_destroy(&node->sensor);
  epos_input_destroy(&node->input);
  epos_device_destroy(&node->dev);

  if (!can_dev->num_references)
    can_destroy(can_dev);

  config_destroy(&node->config);
}

int epos_open(epos_node_p node) {
  if (!epos_device_open(&node->dev) &&
    !epos_motor_setup(&node->motor) &&
    !epos_sensor_setup(&node->sensor) &&
    !epos_input_setup(&node->input))
    return EPOS_ERROR_NONE;
  else
    return EPOS_ERROR_OPEN;
}

int epos_close(epos_node_p node) {
  if (!epos_device_close(&node->dev))
    return EPOS_ERROR_NONE;
  else
    return EPOS_ERROR_CLOSE;
}

float epos_get_position(epos_node_p node) {
  int pos = epos_position_get_actual(&node->dev);
  return epos_gear_to_angle(&node->gear, pos);
}

float epos_get_velocity(epos_node_p node) {
  int vel = epos_velocity_get_average(&node->dev);
  return epos_gear_to_angular_velocity(&node->gear, vel);
}

float epos_get_current(epos_node_p node) {
  return 1e-3*epos_current_get_average(&node->dev);
}

int epos_home(epos_node_p node, double timeout) {
  epos_home_t home;
  epos_home_init(&home,
    config_get_int(&node->config, EPOS_PARAMETER_HOME_METHOD),
    config_get_float(&node->config, EPOS_PARAMETER_HOME_CURRENT),
    deg_to_rad(config_get_float(&node->config, EPOS_PARAMETER_HOME_VELOCITY)),
    deg_to_rad(config_get_float(&node->config, 
      EPOS_PARAMETER_HOME_ACCELERATION)),
    deg_to_rad(config_get_float(&node->config, EPOS_PARAMETER_HOME_POSITION)));
  home.type = config_get_int(&node->config, EPOS_PARAMETER_HOME_TYPE);
  home.offset = deg_to_rad(config_get_float(&node->config, 
    EPOS_PARAMETER_HOME_OFFSET));

  if (!epos_home_start(node, &home)) {
    epos_home_wait(node, timeout);
    return EPOS_ERROR_NONE;
  }
  else
    return EPOS_ERROR_HOME;
}
