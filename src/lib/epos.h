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


#ifndef EPOS_H
#define EPOS_H

#include <tulibs/config.h>

#include "device.h"
#include "sensor.h"
#include "motor.h"
#include "gear.h"
#include "input.h"
#include "control.h"

/** \file epos.h
  * \brief EPOS convenience functions
  */

/** Predefined EPOS constants
  */
#define EPOS_CONFIG_ARG_PREFIX                "epos"

#define EPOS_PARAMETER_ID                     "node-id"
#define EPOS_PARAMETER_RESET                  "node-reset"
#define EPOS_PARAMETER_SENSOR_TYPE            "enc-type"
#define EPOS_PARAMETER_SENSOR_POLARITY        "enc-polarity"
#define EPOS_PARAMETER_SENSOR_PULSES          "enc-pulses"
#define EPOS_PARAMETER_SENSOR_SUPERVISION     "enc-supervision"
#define EPOS_PARAMETER_MOTOR_TYPE             "motor-type"
#define EPOS_PARAMETER_MOTOR_CURRENT          "motor-current"
#define EPOS_PARAMETER_GEAR_TRANSMISSION      "gear-trans"
#define EPOS_PARAMETER_CONTROL_TYPE           "control-type"

#define EPOS_PARAMETER_HOME_METHOD            "home-method"
#define EPOS_PARAMETER_HOME_TYPE              "home-type"
#define EPOS_PARAMETER_HOME_CURRENT           "home-current"
#define EPOS_PARAMETER_HOME_VELOCITY          "home-vel"
#define EPOS_PARAMETER_HOME_ACCELERATION      "home-accel"
#define EPOS_PARAMETER_HOME_OFFSET            "home-offset"
#define EPOS_PARAMETER_HOME_POSITION          "home-pos"

/** Predefined EPOS error codes
  */
#define EPOS_ERROR_NONE                       0
#define EPOS_ERROR_OPEN                       1
#define EPOS_ERROR_CLOSE                      2
#define EPOS_ERROR_HOME                       3

/** \brief Predefined EPOS error descriptions
  */
extern const char* epos_errors[];

/** \brief Predefined EPOS default configuration
  */
extern config_t epos_default_config;

/** \brief Structure defining an EPOS node
  */
typedef struct epos_node_t {
  epos_device_t dev;              //!< The EPOS node device.
  epos_sensor_t sensor;           //!< The EPOS position sensor.
  epos_motor_t motor;             //!< The EPOS motor.
  epos_gear_t gear;               //!< The EPOS gear assembly.
  epos_input_t input;             //!< The EPOS input module.
  epos_control_t control;         //!< The EPOS controller.

  config_t config;                //!< The EPOS configuration parameters.
} epos_node_t, *epos_node_p;

/** \brief Initialize EPOS node
  * \param[in] node The EPOS node to be initialized.
  * \param[in] can_dev The CAN communication device of the EPOS node. If
  *   null, a device will be created from default parameters.
  * \param[in] config The optional EPOS configuration parameters. Can be null.
  */
void epos_init(
  epos_node_p node,
  can_device_p can_dev,
  config_p config);

/** \brief Initialize EPOS node from command line arguments
  * \param[in] node The EPOS node to be initialized.
  * \param[in] argc The number of supplied command line arguments.
  * \param[in] argv The list of supplied command line arguments.
  * \param[in] prefix An optional argument prefix.
  */
void epos_init_arg(
  epos_node_p node,
  int argc,
  char **argv,
  const char* prefix);

/** \brief Destroy EPOS node
  * \note This method automatically destroys unused CAN communication devices.
  * \param[in] node The EPOS node to be destroyed.
  */
void epos_destroy(
  epos_node_p node);

/** \brief Open EPOS node
  * \param[in] node The initialized EPOS node to be opened.
  * \return The resulting error code.
  */
int epos_open(
  epos_node_p node);

/** \brief Close EPOS node
  * \param[in] node The opened EPOS node to be closed.
  * \return The resulting error code.
  */
int epos_close(
  epos_node_p node);

/** \brief Retrieve the angular position of an EPOS node
  * \param[in] node The opened EPOS node to retrieve the angular position for.
  * \return The angular position of the specified EPOS node in [rad].
  */
float epos_get_position(
  epos_node_p node);

/** \brief Retrieve the angular velocity of an EPOS node
  * \param[in] node The opened EPOS node to retrieve the angular velocity for.
  * \return The angular velocity of the specified EPOS node in [rad/s].
  */
float epos_get_velocity(
  epos_node_p node);

/** \brief Retrieve the current of an EPOS node
  * \param[in] node The opened EPOS node to retrieve the current for.
  * \return The current of the specified EPOS node in [A].
  */
float epos_get_current(
  epos_node_p node);

/** \brief Home an EPOS node from configuration settings
  * \param[in] node The opened EPOS node to be homed.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting error code.
  */
int epos_home(
  epos_node_p node,
  double timeout);

#endif
