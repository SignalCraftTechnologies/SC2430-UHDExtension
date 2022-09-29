//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "sc2430_expert.hpp"
#include "filter_specification.hpp"
#include "sc2430_constants.hpp"
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/utils/log.hpp>

using namespace uhd;

namespace {

std::string to_trx_str(const uhd::direction_t trx)
{
    if (trx == uhd::RX_DIRECTION)
        return "RX";
    else if (trx == uhd::TX_DIRECTION)
        return "TX";
    else
        UHD_THROW_INVALID_CODE_PATH();
}

std::string to_path_str(const scm::path& path)
{
    std::stringstream stream;
    stream << ", CH" << path.chan << "-" << to_trx_str(path.trx);
    return stream.str();
}

} // namespace

namespace scm {

void frequency_programming_expert::resolve(void)
{
    UHD_LOG_TRACE(scm::NAME, "frequency_programming_expert::resolve");

    std::string band = _band_select_in;

    if (band.empty())
        band = get_default_band(_scm_frequency_in);

    const filter_spec filter(band, _scm_frequency_in);
    UHD_LOG_DEBUG(scm::NAME,
        "Programming filter: "
            << "band \"" << band << "\", frequency " << filter.coerced_frequency << " Hz ("
            << filter.frequency_idx << ")" << to_path_str(_path));

    if (_path.trx == RX_DIRECTION)
        _scm_frequency_out = _radio->set_rx_frequency(filter.coerced_frequency, _path.chan);
    else
        _scm_frequency_out = _radio->set_tx_frequency(filter.coerced_frequency, _path.chan);

    _band_select_out = band;
    _phy->set_filter_spec(_path, filter);
}

void gain_programming_expert::resolve(void)
{
    UHD_LOG_TRACE(scm::NAME, "gain_programming_expert::resolve");
    UHD_LOG_DEBUG(scm::NAME, "Programming gain: " << _scm_gain_in << " dB" << to_path_str(_path));

    uhd::gain_range_t gain_range(-128, 127, 1);

    const int8_t gain = gain_range.clip(_scm_gain_in, true);
    _phy->set_gain(_path, gain);
    _scm_gain_out = static_cast<double>(gain);
}

// Resolution of gain:
// For RX:
//   - For best performance, utilize the SCM gain while minimizing the ZBX gain.
//   - Once the SCM gain is maximized, adjust the ZBX gain as required.
// For TX:
//   - For best performance, limit the ZBX output power to TX_IN_TYP_MAX_POWER.
//   - Set additional gain using the SCM once TX_IN_TYP_MAX_POWER is exceeded.
void gain_expert::resolve(void)
{
    UHD_LOG_TRACE(scm::NAME, "gain_expert::resolve");
    UHD_LOG_DEBUG(scm::NAME, "Setting gain: " << _gain_in << " dB" << to_path_str(_path));

    const auto chan = _path.chan;
    if (_path.trx == RX_DIRECTION) {
        // Set the ZBX to Minimum Gain
        double zbx_gain = _radio->get_rx_gain_range(chan).start();
        // For best performance, utilize the SCM gain before the ZBX gain.
        _scm_gain_out = _phy->get_rx_gain_range(chan).clip(_gain_in - zbx_gain, true);

        // Adjust the ZBX gain for the actual SCM gain
        zbx_gain = _gain_in - _scm_gain_out;
        zbx_gain = _radio->set_rx_gain(zbx_gain, chan);

        _gain_out = zbx_gain + _scm_gain_out;
    } else {
        const auto scm_gain_range = _phy->get_tx_gain_range(chan);

        // Set the SCM to Minimum Gain
        _scm_gain_out = scm_gain_range.start();

        // For best performance, limit the ZBX max output power.
        const double zbx_pref     = _radio->get_tx_power_reference(chan);
        const double zbx_gain_max = TX_IN_TYP_MAX_POWER - zbx_pref + _radio->get_tx_gain(chan);
        // Limit the ZBX gain to ensure the power does not exceed the max.
        double zbx_gain = _gain_in - _scm_gain_out;
        zbx_gain        = (zbx_gain > zbx_gain_max) ? zbx_gain_max : zbx_gain;
        zbx_gain        = _radio->set_tx_gain(zbx_gain, chan);

        // Adjust the SCM gain for the actual ZBX gain
        _scm_gain_out = scm_gain_range.clip(_gain_in - zbx_gain, true);

        // If required, additional gain is available from the ZBX.
        double zbx_extra_gain = _gain_in - zbx_gain - _scm_gain_out;
        if (zbx_extra_gain > _radio->get_tx_gain_range(chan).step()) {
            const double zbx_gain_extra_max = (TX_IN_ABS_MAX_POWER - TX_IN_TYP_MAX_POWER);
            zbx_extra_gain = (zbx_extra_gain > zbx_gain_extra_max) ? zbx_gain_extra_max
                                                                   : zbx_extra_gain;
            zbx_gain       = _radio->set_tx_gain(zbx_gain + zbx_extra_gain, chan);
        }
        _gain_out = zbx_gain + _scm_gain_out;
    }
    _radio->get_tree()->access<bool>(_power_ref_path).set(false);
}

// Resolution of power reference:
// For RX:
//   - For best performance, utilize the SCM gain while maximizing the ZBX pref.
//   - Decrease the SCM gain once the ZBX power reference is maximized.
// For TX:
//   - For best performance, limit the ZBX output power to TX_IN_TYP_MAX_POWER.
//   - Set additional gain using the SCM once TX_IN_TYP_MAX_POWER is exceeded.
void power_ref_expert::resolve(void)
{
    UHD_LOG_TRACE(scm::NAME, "power_ref_expert::resolve");

    const bool is_power_ref_mode = _radio->get_tree()->access<bool>(_power_ref_path).get();
    if (_freq_in.is_dirty() && !_power_ref_in.is_dirty() && !is_power_ref_mode) {
        return;
    }
    UHD_LOG_DEBUG(scm::NAME, "Setting pref: " << _power_ref_in << " dBm" << to_path_str(_path));

    const auto chan = _path.chan;
    if (_path.trx == uhd::direction_t::RX_DIRECTION) {
        const auto scm_gain_range = _phy->get_rx_gain_range(chan);

        // Set SCM to Maximum Gain
        _scm_gain_out = scm_gain_range.stop();

        // Calculate the desired ZBX power ref with the SCM at max gain
        const double zbx_desired_pref = _power_ref_in + _scm_gain_out;
        _radio->set_rx_power_reference(zbx_desired_pref, chan);
        const double zbx_pref = _radio->get_rx_power_reference(chan);

        // Decrease the SCM gain if required
        const double gain_adjust = zbx_desired_pref - zbx_pref;
        if (gain_adjust > scm_gain_range.step())
            _scm_gain_out = _scm_gain_out - scm_gain_range.clip(gain_adjust, true);

        _power_ref_out = zbx_pref - _scm_gain_out;
    } else {
        const auto gain_range = _phy->get_tx_gain_range(chan);

        // Set the SCM to Minimum Gain
        _scm_gain_out = _phy->get_tx_gain_range(chan).start();

        // Calculate the desired ZBX power ref with the SCM at min gain
        const double zbx_desired_pref = _power_ref_in - _scm_gain_out;
        // For best performance, limit the ZBX max output power.
        double zbx_pref = (zbx_desired_pref > TX_IN_TYP_MAX_POWER) ? TX_IN_TYP_MAX_POWER
                                                                   : zbx_desired_pref;
        _radio->set_tx_power_reference(zbx_pref, chan);
        zbx_pref = _radio->get_tx_power_reference(chan);

        // Set the final SCM gain required
        _scm_gain_out = gain_range.clip(_power_ref_in - zbx_pref, true);

        // If required, additional power is available from the ZBX.
        double zbx_extra_pref = _power_ref_in - zbx_pref - _scm_gain_out;
        if (zbx_extra_pref > _radio->get_tx_power_range(chan).step()) {
            const double zbx_pref_extra_max = (TX_IN_ABS_MAX_POWER - TX_IN_TYP_MAX_POWER);
            zbx_extra_pref = (zbx_extra_pref > zbx_pref_extra_max) ? zbx_pref_extra_max
                                                                   : zbx_extra_pref;
            _radio->set_tx_power_reference(zbx_pref + zbx_extra_pref, chan);
            zbx_pref = _radio->get_tx_power_reference(chan);
        }
        _power_ref_out = zbx_pref + _scm_gain_out;
    }
    _radio->get_tree()->access<bool>(_power_ref_path).set(true);
}

} // namespace scm
