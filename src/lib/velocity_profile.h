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


#ifndef EPOS_VELOCITY_PROFILE_H
#define EPOS_VELOCITY_PROFILE_H

#include "profile.h"

/** \file velocity_profile.h
  * \brief EPOS velocity profile mode functions
  */

/** Predefined EPOS velocity profile control constants
  */
#define EPOS_VELOCITY_PROFILE_INDEX_TARGET            0x60FF

#define EPOS_VELOCITY_PROFILE_CONTROL_SET             0x007F

/** \brief Structure defining an EPOS velocity profile control operation
  */
typedef struct epos_velocity_profile_t {
  float target_value;            //!< The target velocity in [rad/s].
  float acceleration;            //!< The profile acceleration in [rad/s^2].
  float deceleration;            //!< The profile acceleration in [rad/s^2].

  epos_profile_type_t type;      //!< The velocity profile type.
} epos_velocity_profile_t, *epos_velocity_profile_p;

/** \brief Initialize EPOS velocity profile control operation
  * \param[in] profile The EPOS velocity profile control operation to be
  *   initialized.
  * \param[in] target_value The target velocity in [rad/s].
  * \param[in] acceleration The profile acceleration in [rad/s^2].
  * \param[in] deceleration The profile deceleration in [rad/s^2].
  * \param[in] type The velocity profile type.
  */
void epos_velocity_profile_init(
  epos_velocity_profile_p profile,
  float target_value,
  float acceleration,
  float deceleration,
  epos_profile_type_t type);

/** \brief Start EPOS velocity profile control operation
  * \param[in] node The EPOS node to start the velocity profile control
  *   operation for.
  * \param[in] profile The EPOS velocity profile control operation to be
  *   started.
  * \return The resulting device error code.
  */
int epos_velocity_profile_start(
  epos_node_p node,
  epos_velocity_profile_p profile);

/** \brief Stop EPOS velocity profile control operation
  * \param[in] node The EPOS node to stop the velocity profile control
  *   operation for.
  * \return The resulting device error code.
  */
int epos_velocity_profile_stop(
  epos_node_p node);

/** \brief Set the velocity profile target velocity of an EPOS device
  * \param[in] dev The EPOS device to set the target velocity for.
  * \param[in] velocity The target velocity for the specified EPOS
  *   device in [vu].
  * \return The resulting device error code.
  */
int epos_velocity_profile_set_target(
  epos_device_p dev,
  int velocity);

#endif
