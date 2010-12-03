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


#ifndef EPOS_POSITION_PROFILE_H
#define EPOS_POSITION_PROFILE_H

#include "profile.h"

/** \file position_profile.h
  * \brief EPOS position profile mode functions
  */

/** Predefined EPOS position profile control constants
  */
#define EPOS_POSITION_PROFILE_INDEX_TARGET            0x607A
#define EPOS_POSITION_PROFILE_INDEX_VELOCITY          0x6081

#define EPOS_POSITION_PROFILE_CONTROL_SET_ABSOLUTE    0x003F
#define EPOS_POSITION_PROFILE_CONTROL_SET_RELATIVE    0x007F

/** \brief Structure defining an EPOS position profile control operation
  */
typedef struct epos_position_profile_t {
  float target_value;            //!< The target position in [rad].
  float velocity;                //!< The profile velocity in [rad/s].
  float acceleration;            //!< The profile acceleration in [rad/s^2].
  float deceleration;            //!< The profile acceleration in [rad/s^2].

  int relative;                  //!< The profile position is relative.
  epos_profile_type_t type;      //!< The position profile type.

  float start_value;             //!< The start position of the profile in [s].
  double start_time;             //!< The start time of the profile in [s].
} epos_position_profile_t, *epos_position_profile_p;

/** \brief Initialize EPOS position profile control operation
  * \param[in] profile The EPOS position profile control operation to be
  *   initialized.
  * \param[in] target_value The target position in [rad].
  * \param[in] velocity The profile velocity in [rad/s].
  * \param[in] acceleration The profile acceleration in [rad/s^2].
  * \param[in] deceleration The profile deceleration in [rad/s^2].
  * \param[in] type The position profile type.
  */
void epos_position_profile_init(
  epos_position_profile_p profile,
  float target_value,
  float velocity,
  float acceleration,
  float deceleration,
  epos_profile_type_t type);

/** \brief Start EPOS position profile control operation
  * \param[in] node The EPOS node to start the position profile control
  *   operation for.
  * \param[in] profile The EPOS position profile control operation to be
  *   started.
  * \return The resulting device error code.
  */
int epos_position_profile_start(
  epos_node_p node,
  epos_position_profile_p profile);

/** \brief Stop EPOS position profile control operation
  * \param[in] node The EPOS node to stop the position profile control
  *   operation for.
  * \return The resulting device error code.
  */
int epos_position_profile_stop(
  epos_node_p node);

/** \brief Estimate the relative position of an EPOS position profile
  * \param[in] profile The EPOS position profile control operation to
  *   estimate the relative position of.
  * \param[in] time The absolute time to estimate the relative position
  *    at in [s].
  * \return The estimated relative position in [rad].
  */
float epos_position_profile_estimate(
  epos_position_profile_p profile,
  double time);

/** \brief Set the position profile target position of an EPOS device
  * \param[in] dev The EPOS device to set the target position for.
  * \param[in] position The target position for the specified EPOS
  *   device in [pu].
  * \return The resulting device error code.
  */
int epos_position_profile_set_target(
  epos_device_p dev,
  int position);

/** \brief Set the position profile velocity of an EPOS device
  * \param[in] dev The EPOS device to set the profile velocity for.
  * \param[in] velocity The profile velocity for the specified EPOS
  *   device in [vu].
  * \return The resulting device error code.
  */
int epos_position_profile_set_velocity(
  epos_device_p dev,
  unsigned int velocity);

#endif
