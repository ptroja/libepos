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


#ifndef EPOS_GEAR_H
#define EPOS_GEAR_H

#include "sensor.h"

/** \file gear.h
  * \brief EPOS gear functions
  */

/** \brief Structure defining an EPOS gear assembly
  */
typedef struct epos_gear_t {
  epos_sensor_p sensor;            //!< The EPOS position sensor of the gear.

  float transmission;              //!< The gear transmission factor.
} epos_gear_t, *epos_gear_p;

/** \brief Initialize an EPOS gear assembly
  * \param[in] gear The EPOS gear assembly to be initialized.
  * \param[in] sensor The EPOS position sensor the gear assembly is
  *   connected to.
  * \param[in] transmission The transmission factor of the EPOS gear to be
  *   initialized.
  */
void epos_gear_init(
  epos_gear_p gear,
  epos_sensor_p sensor,
  float transmission);

/** \brief Destroy an EPOS gear assembly
  * \param[in] gear The EPOS gear assembly to be destroyed.
  */
void epos_gear_destroy(
  epos_gear_p gear);

/** \brief Convert EPOS position units to radian space angle
  * \param[in] gear The EPOS gear assembly to be used for conversion.
  * \param[in] position The number of EPOS position units to be converted
  *   into an angle.
  * \return The angle corresponding to the specified number of EPOS
  *   position units in [rad].
  */
float epos_gear_to_angle(
  epos_gear_p gear,
  int position);

/** \brief Convert radian space angle to EPOS position units
  * \param[in] gear The EPOS gear assembly to be used for conversion.
  * \param[in] angle The angle to be converted into EPOS position
  *   units in [rad].
  * \return The number of EPOS position units corresponding to the
  *   specified angle.
  */
int epos_gear_from_angle(
  epos_gear_p gear,
  float angle);

/** \brief Convert EPOS velocity units to radian space angular velocity
  * \param[in] gear The EPOS gear assembly to be used for conversion.
  * \param[in] velocity The number of EPOS velocity units to be converted
  *   into an angular velocity.
  * \return The angular velocity corresponding to the specified number of
  *   EPOS velocity units in [rad/s].
  */
float epos_gear_to_angular_velocity(
  epos_gear_p gear,
  int velocity);

/** \brief Convert radian space angular velocity to EPOS velocity units
  * \param[in] gear The EPOS gear assembly to be used for conversion.
  * \param[in] angular_vel The angular velocity to be converted into EPOS
  *   velocity units in [rad/s].
  * \return The number of EPOS velocity units corresponding to the
  *   specified angular velocity.
  */
int epos_gear_from_angular_velocity(
  epos_gear_p gear,
  float angular_vel);

/** \brief Convert EPOS acceleration units to radian space angular acceleration
  * \param[in] gear The EPOS gear assembly to be used for conversion.
  * \param[in] acceleration The number of EPOS acceleration units to be
  *   converted into an angular acceleration.
  * \return The angular acceleration corresponding to the specified number of
  *   EPOS velocity units in [rad/s^2].
  */
float epos_gear_to_angular_acceleration(
  epos_gear_p gear,
  int acceleration);

/** \brief Convert radian space angular acceleration to EPOS acceleration units
  * \param[in] gear The EPOS gear assembly to be used for conversion.
  * \param[in] angular_acc The angular acceleration to be converted into EPOS
  *   acceleration units in [rad/s^2].
  * \return The number of EPOS acceleration units corresponding to the
  *   specified angular acceleration.
  */
int epos_gear_from_angular_acceleration(
  epos_gear_p gear,
  float angular_acc);

#endif
