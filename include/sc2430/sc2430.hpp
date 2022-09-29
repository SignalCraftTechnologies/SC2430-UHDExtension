//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/extension/extension.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc/rf_control/core_iface.hpp>
#include <uhd/utils/noncopyable.hpp>
#include <memory>

namespace scm {

/*! Interface for SC2430
 *
 * This interface contains all methods related directly to the SC2430.
 */
class sc2430 : public uhd::extension::extension
{
public:
    using sptr = std::shared_ptr<sc2430>;

    virtual ~sc2430() = default;

    /*!
     * Read a register from the general purpose protocol of the SC2430.
     * Not intended for typical use.
     *
     * \param address The register address to read.
     * \returns The value of the register specified.
     */
    virtual uint16_t read_reg(uint8_t address) = 0;

    /*!
     * Writes a register to the general purpose control of the SC2430.
     * Not intended for typical use.
     *
     * \param address The register address to write.
     * \param value Value to write to the register address specified.
     */
    virtual void write_reg(uint8_t address, uint16_t value) = 0;

    /*!
     * Make an instance of the sc2430.
     * This assumes the SCM is connected to the matching  X410 radio channel and slot.
     * Additionally, the GPIO HDMI cables must be connected between devices.
     * 
     * \param fargs Factory args that hold pointers to radio control and motherboard
     * controller
     * \returns Smart pointer to sc2430 class object
     */
    static sptr make(uhd::extension::extension::factory_args fargs);

    /*!
     * Enables or disables the gain updates on the signal conditioning accessory. If
     * updates are enabled, attenuation/gain settings will be applied to the SC2430 hardware.
     * If updates are disabled the configuration will be written to the SC2430, however, they
     * will not be applied until the gain updates are re-enabled. The attenuator latches are
     * available per channel and therefore can be configured individually.
     *
     * \param enabled Enables or disables gain update for the channel specified
     * \param chan The channel on which the gain updates should be enabled/disabled
     */
    virtual void set_gain_updates(const bool enabled, const size_t chan) = 0;

    /*!
     * Get the current status of the gain update latches of the SC2430 hardware.
     * 
     * \param chan The channel of which to get the gain updates enabled/disabled state
     * \returns The enable or disable state of the gain updates for the channel specified
     */
    virtual bool get_gain_updates(const size_t chan) = 0;
};

} // namespace scm
