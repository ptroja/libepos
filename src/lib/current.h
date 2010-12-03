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


#ifndef EPOS_CURRENT_H
#define EPOS_CURRENT_H

#include "epos.h"

/** \file current.h
  * \brief EPOS current mode functions
  */

/** Predefined EPOS current control constants
  */
#define EPOS_CURRENT_INDEX_SETTING_VALUE           0x2030
#define EPOS_CURRENT_INDEX_ACTUAL_VALUE            0x6078
#define EPOS_CURRENT_INDEX_AVERAGE_VALUE           0x2027
#define EPOS_CURRENT_INDEX_CONTROL_PARAMETERS      0x60F6
#define EPOS_CURRENT_SUBINDEX_P_GAIN               0x01
#define EPOS_CURRENT_SUBINDEX_I_GAIN               0x02

/** \brief Structure defining an EPOS current control configuration
  */
typedef struct epos_current_config_t {
  short p_gain;         //!< The current controller's P-gain.
  short i_gain;         //!< The current controller's I-gain.
} epos_current_config_t, *epos_current_config_p;

/** \brief Structure defining an EPOS current control operation
  */
typedef struct epos_current_t {
  float target_value;          //!< The target current in [A].
} epos_current_t, *epos_current_p;

/** \brief Initialize EPOS current control operation
  * \param[in] current The EPOS current control operation to be initialized.
  * \param[in] target_value The target current in [A].
  */
void epos_current_init(
  epos_current_p current,
  float target_value);

/** \brief Setup EPOS current control
  * \param[in] node The EPOS node to setup current control for.
  * \param[in] config The configuration to be used for setting up the
  *   EPOS current controller.
  * \return The resulting device error code.
  */
int epos_current_setup(
  epos_node_p node,
  epos_current_config_p config);

/** \brief Start EPOS current control operation
  * \param[in] node The EPOS node to start the current control operation for.
  * \param[in] current The EPOS current control operation to be started.
  * \return The resulting device error code.
  */
int epos_current_start(
  epos_node_p node,
  epos_current_p current);

/** \brief Stop EPOS current control operation
  * \param[in] node The EPOS node to stop the current control operation for.
  * \return The resulting device error code.
  */
int epos_current_stop(
  epos_node_p node);

/** \brief Retrieve the actual current of an EPOS device
  * \param[in] dev The EPOS device to retrieve the actual current for.
  * \return The actual current of the specified EPOS device in [mA].
  */
short epos_current_get_actual(
  epos_device_p dev);

/** \brief Retrieve the average current of an EPOS device
  * \param[in] dev The EPOS device to retrieve the average current for.
  * \return The average current of the specified EPOS device in [mA].
  */
short epos_current_get_average(
  epos_device_p dev);

/** \brief Set the demanded current of an EPOS device
  * \param[in] dev The EPOS device to set the demanded current for.
  * \param[in] current The demanded current for the specified EPOS
  *   device in [mA].
  * \return The resulting device error code.
  */
int epos_current_set_demand(
  epos_device_p dev,
  short current);

/** \brief Retrieve the demanded current of an EPOS device
  * \param[in] dev The EPOS device to retrieve the demanded current for.
  * \return The demanded current of the specified EPOS device in [mA].
  */
short epos_current_get_demand(
  epos_device_p dev);

/** \brief Set the current control P-gain of an EPOS device
  * \param[in] dev The EPOS device to set the current control P-gain for.
  * \param[in] p_gain The current control P-gain for the specified EPOS
  *   device.
  * \return The resulting device error code.
  */
int epos_current_set_p_gain(
  epos_device_p dev,
  short p_gain);

/** \brief Set the current control I-gain of an EPOS device
  * \param[in] dev The EPOS device to set the current control I-gain for.
  * \param[in] i_gain The current control I-gain for the specified EPOS
  *   device.
  * \return The resulting device error code.
  */
int epos_current_set_i_gain(
  epos_device_p dev,
  short i_gain);

#endif
