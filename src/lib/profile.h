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


#ifndef EPOS_PROFILE_H
#define EPOS_PROFILE_H

#include "epos.h"

/** \file profile.h
  * \brief EPOS profile generic functions
  */

/** Predefined EPOS profile control constants
  */
#define EPOS_PROFILE_INDEX_ACCELERATION       0x6083
#define EPOS_PROFILE_INDEX_DECELERATION       0x6084
#define EPOS_PROFILE_INDEX_TYPE               0x6086

#define EPOS_PROFILE_STATUS_REACHED           0x0400

/** \brief EPOS motion profile types
  */
typedef enum {
  epos_profile_linear = 0,
  epos_profile_sinusoidal = 1
} epos_profile_type_t;

/** \brief Wait for completion of an EPOS motion profile
  * \param[in] node The EPOS node to complete the motion profile.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting device error code.
  */
int epos_profile_wait(
  epos_node_p node,
  double timeout);

/** \brief Set the profile acceleration of an EPOS device
  * \param[in] dev The EPOS device to set the profile acceleration for.
  * \param[in] acceleration The profile acceleration for the specified
  *   EPOS device in [au].
  * \return The resulting device error code.
  */
int epos_profile_set_acceleration(
  epos_device_p dev,
  unsigned int acceleration);

/** \brief Set the profile deceleration of an EPOS device
  * \param[in] dev The EPOS device to set the profile deceleration for.
  * \param[in] deceleration The profile deceleration for the specified
  *   EPOS device in [au].
  * \return The resulting device error code.
  */
int epos_profile_set_deceleration(
  epos_device_p dev,
  unsigned int deceleration);

/** \brief Set the motion profile type of an EPOS device
  * \param[in] dev The EPOS device to set the motion profile type for.
  * \param[in] type The motion profile type for the specified EPOS device.
  * \return The resulting device error code.
  */
int epos_profile_set_type(
  epos_device_p dev,
  epos_profile_type_t type);

#endif
