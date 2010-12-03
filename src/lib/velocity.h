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


#ifndef EPOS_VELOCITY_H
#define EPOS_VELOCITY_H

#include "epos.h"

/** \file velocity.h
  * \brief EPOS velocity mode functions
  */

/** Predefined EPOS velocity control constants
  */
#define EPOS_VELOCITY_INDEX_SETTING_VALUE           0x206B
#define EPOS_VELOCITY_INDEX_DEMAND_VALUE            0x606B
#define EPOS_VELOCITY_INDEX_ACTUAL_VALUE            0x606C
#define EPOS_VELOCITY_INDEX_AVERAGE_VALUE           0x2028
#define EPOS_VELOCITY_INDEX_CONTROL_PARAMETERS      0x60F9
#define EPOS_VELOCITY_SUBINDEX_P_GAIN               0x01
#define EPOS_VELOCITY_SUBINDEX_I_GAIN               0x02

/** \brief Structure defining an EPOS velocity control configuration
  */
typedef struct epos_velocity_config_t {
  short p_gain;         //!< The velocity controller's P-gain.
  short i_gain;         //!< The velocity controller's I-gain.
} epos_velocity_config_t, *epos_velocity_config_p;

/** \brief Structure defining an EPOS velocity control operation
  */
typedef struct epos_velocity_t {
  float target_value;            //!< The target angular velocity in [rad/s].
} epos_velocity_t, *epos_velocity_p;

/** \brief Initialize EPOS velocity control operation
  * \param[in] velocity The EPOS velocity control operation to be initialized.
  * \param[in] target_value The target angular velocity in [rad/s].
  */
void epos_velocity_init(
  epos_velocity_p velocity,
  float target_value);

/** \brief Setup EPOS velocity control
  * \param[in] node The EPOS node to setup velocity control for.
  * \param[in] config The configuration to be used for setting up the
  *   EPOS velocity controller.
  * \return The resulting device error code.
  */
int epos_velocity_setup(
  epos_node_p node,
  epos_velocity_config_p config);

/** \brief Start EPOS velocity control operation
  * \param[in] node The EPOS node to start the velocity control operation for.
  * \param[in] velocity The EPOS velocity control operation to be started.
  * \return The resulting device error code.
  */
int epos_velocity_start(
  epos_node_p node,
  epos_velocity_p velocity);

/** \brief Stop EPOS velocity control operation
  * \param[in] node The EPOS node to stop the velocity control operation for.
  * \return The resulting device error code.
  */
int epos_velocity_stop(
  epos_node_p node);

/** \brief Update the velocity control operation of an EPOS device
  * \param[in] node The EPOS node to update the velocity control 
  *   operation for.
  * \param[in] velocity The updated velocity control operation.
  * \return The resulting device error code.
  */
int epos_velocity_update(
  epos_node_p node,
  epos_velocity_p velocity);

/** \brief Retrieve the actual velocity of an EPOS device
  * \param[in] dev The EPOS device to retrieve the actual velocity for.
  * \return The actual velocity of the specified EPOS device in [vu].
  */
int epos_velocity_get_actual(
  epos_device_p dev);

/** \brief Retrieve the average velocity of an EPOS device
  * \param[in] dev The EPOS device to retrieve the average velocity for.
  * \return The average velocity of the specified EPOS device in [vu].
  */
int epos_velocity_get_average(
  epos_device_p dev);

/** \brief Set the demanded velocity of an EPOS device
  * \param[in] dev The EPOS device to set the demanded velocity for.
  * \param[in] velocity The demanded velocity for the specified EPOS
  *   device in [vu].
  * \return The resulting device error code.
  */
int epos_velocity_set_demand(
  epos_device_p dev,
  int velocity);

/** \brief Retrieve the demanded velocity of an EPOS device
  * \param[in] dev The EPOS device to retrieve the demanded velocity for.
  * \return The demanded velocity of the specified EPOS device in [vu].
  */
int epos_velocity_get_demand(
  epos_device_p dev);

/** \brief Set the velocity control P-gain of an EPOS device
  * \param[in] dev The EPOS device to set the velocity control P-gain for.
  * \param[in] p_gain The velocity control P-gain for the specified EPOS
  *   device.
  * \return The resulting device error code.
  */
int epos_velocity_set_p_gain(
  epos_device_p dev,
  short p_gain);

/** \brief Set the velocity control I-gain of an EPOS device
  * \param[in] dev The EPOS device to set the velocity control I-gain for.
  * \param[in] i_gain The velocity control I-gain for the specified EPOS
  *   device.
  * \return The resulting device error code.
  */
int epos_velocity_set_i_gain(
  epos_device_p dev,
  short i_gain);

#endif
