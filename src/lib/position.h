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


#ifndef EPOS_POSITION_H
#define EPOS_POSITION_H

#include "epos.h"

/** \file position.h
  * \brief EPOS position mode functions
  */

/** Predefined EPOS position control constants
  */
#define EPOS_POSITION_INDEX_SETTING_VALUE           0x2062
#define EPOS_POSITION_INDEX_DEMAND_VALUE            0x6062
#define EPOS_POSITION_INDEX_ACTUAL_VALUE            0x6064
#define EPOS_POSITION_INDEX_MAX_FOLLOWING_ERROR     0x6065
#define EPOS_POSITION_INDEX_SOFTWARE_LIMIT          0x607D
#define EPOS_POSITION_SUBINDEX_NEG_LIMIT            0x01
#define EPOS_POSITION_SUBINDEX_POS_LIMIT            0x02
#define EPOS_POSITION_INDEX_CONTROL_PARAMETERS      0x60FB
#define EPOS_POSITION_SUBINDEX_P_GAIN               0x01
#define EPOS_POSITION_SUBINDEX_I_GAIN               0x02
#define EPOS_POSITION_SUBINDEX_D_GAIN               0x03
#define EPOS_POSITION_SUBINDEX_VELOCITY_FACTOR      0x04
#define EPOS_POSITION_SUBINDEX_ACCELERATION_FACTOR  0x05

/** \brief Structure defining an EPOS position control configuration
  */
typedef struct epos_position_config_t {
  short p_gain;         //!< The position controller's P-gain.
  short i_gain;         //!< The position controller's I-gain.
  short d_gain;         //!< The position controller's D-gain.
  short vel_factor;     //!< The controller's velocity feed-forward factor.
  short acc_factor;     //!< The controller's acceleration feed-forward factor.
} epos_position_config_t, *epos_position_config_p;

/** \brief Structure defining an EPOS position control operation
  */
typedef struct epos_position_t {
  float target_value;   //!< The target position in [rad].

  float min_value;      //!< The minimum position limit in [rad].
  float max_value;      //!< The maximum position limit in [rad].
  float max_error;      //!< The maximum position following error in [rad].
} epos_position_t, *epos_position_p;

/** \brief Initialize EPOS position control operation
  * \param[in] position The EPOS position control operation to be initialized.
  * \param[in] target_value The target position in [rad].
  */
void epos_position_init(
  epos_position_p position,
  float target_value);

/** \brief Initialize EPOS position control operation involving limits
  * \param[in] position The EPOS position control operation to be initialized.
  * \param[in] target_value The target position in [rad].
  * \param[in] min_value The minimum position angle in [rad].
  * \param[in] max_value The maximum position angle in [rad].
  * \param[in] max_error The maximum following error in [rad].
  */
void epos_position_init_limits(
  epos_position_p position,
  float target_value,
  float min_value,
  float max_value,
  float max_error);

/** \brief Setup EPOS position control
  * \param[in] node The EPOS node to setup position control for.
  * \param[in] config The configuration to be used for setting up the
  *   EPOS position controller.
  * \return The resulting device error code.
  */
int epos_position_setup(
  epos_node_p node,
  epos_position_config_p config);

/** \brief Start EPOS position control operation
  * \param[in] node The EPOS node to start the position control operation for.
  * \param[in] position The EPOS position control operation to be started.
  * \return The resulting device error code.
  */
int epos_position_start(
  epos_node_p node,
  epos_position_p position);

/** \brief Stop EPOS position control operation
  * \param[in] node The EPOS node to stop the position control operation for.
  * \return The resulting device error code.
  */
int epos_position_stop(
  epos_node_p node);

/** \brief Update the position control operation of an EPOS device
  * \param[in] node The EPOS node to update the position control operation for.
  * \param[in] position The update position control operation.
  * \return The resulting device error code.
  */
int epos_position_update(
  epos_node_p node,
  epos_position_p position);

/** \brief Set EPOS software position limits
  * \param[in] dev The EPOS device to set the software position limits for.
  * \param[in] min_pos The minimal position limit of the EPOS device in [pu].
  * \param[in] max_pos The maximal position limit of the EPOS device in [pu].
  * \return The resulting device error code.
  */
int epos_position_set_limits(
  epos_device_p dev,
  int min_pos,
  int max_pos);

/** \brief Retrieve the actual position of an EPOS device
  * \param[in] dev The EPOS device to retrieve the actual position for.
  * \return The actual position of the specified EPOS device in [pu].
  */
int epos_position_get_actual(
  epos_device_p dev);

/** \brief Set the demanded position of an EPOS device
  * \param[in] dev The EPOS device to set the demanded position for.
  * \param[in] position The demanded position for the specified EPOS
  *   device in [pu].
  * \return The resulting device error code.
  */
int epos_position_set_demand(
  epos_device_p dev,
  int position);

/** \brief Retrieve the demanded position of an EPOS device
  * \param[in] dev The EPOS device to retrieve the demanded position for.
  * \return The demanded position of the specified EPOS device in [pu].
  */
int epos_position_get_demand(
  epos_device_p dev);

/** \brief Set the maximum position following error of an EPOS device
  * \param[in] dev The EPOS device to set the maximum following error for.
  * \param[in] max_error The maximum following error for the specified EPOS
  *   device in [pu].
  * \return The resulting device error code.
  */
int epos_position_set_max_error(
  epos_device_p dev,
  unsigned int max_error);

/** \brief Set the position control P-gain of an EPOS device
  * \param[in] dev The EPOS device to set the position control P-gain for.
  * \param[in] p_gain The position control P-gain for the specified EPOS
  *   device.
  * \return The resulting device error code.
  */
int epos_position_set_p_gain(
  epos_device_p dev,
  short p_gain);

/** \brief Set the position control I-gain of an EPOS device
  * \param[in] dev The EPOS device to set the position control I-gain for.
  * \param[in] i_gain The position control I-gain for the specified EPOS
  *   device.
  * \return The resulting device error code.
  */
int epos_position_set_i_gain(
  epos_device_p dev,
  short i_gain);

/** \brief Set the position control D-gain of an EPOS device
  * \param[in] dev The EPOS device to set the position control D-gain for.
  * \param[in] d_gain The position control D-gain for the specified EPOS
  *   device.
  * \return The resulting device error code.
  */
int epos_position_set_d_gain(
  epos_device_p dev,
  short d_gain);

/** \brief Set the velocity feed-forward factor of an EPOS device
  * \param[in] dev The EPOS device to set the velocity feed-forward factor for.
  * \param[in] vel_factor The velocity feed-forward factor for the specified
  *   EPOS device.
  * \return The resulting device error code.
  */
int epos_position_set_velocity_factor(
  epos_device_p dev,
  short vel_factor);

/** \brief Set the acceleration feed-forward factor of an EPOS device
  * \param[in] dev The EPOS device to set the acceleration feed-forward
  *   factor for.
  * \param[in] acc_factor The acceleration feed-forward factor for the
  *   specified EPOS device.
  * \return The resulting device error code.
  */
int epos_position_set_acceleration_factor(
  epos_device_p dev,
  short acc_factor);

#endif
