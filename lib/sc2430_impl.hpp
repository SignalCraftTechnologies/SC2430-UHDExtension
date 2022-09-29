//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include "sc2430_constants.hpp"
#include "sc2430_phy.hpp"
#include <uhd/experts/expert_factory.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc/rf_control/antenna_iface.hpp>
#include <uhd/rfnoc/rf_control/nameless_gain_mixin.hpp>
#include <uhd/types/serial.hpp>
#include <sc2430/sc2430.hpp>
#include <string>
#include <vector>

namespace scm {

class sc2430_impl : public sc2430,
                    public uhd::rfnoc::rf_control::antenna_radio_control_mixin,
                    public uhd::rfnoc::rf_control::nameless_gain_mixin
{
private:
    uhd::rfnoc::radio_control::sptr _radio;
    uhd::experts::expert_container::sptr _expert_container;
    uhd::property_tree::sptr _tree;
    phy::sptr _phy;

public:
    using sptr = std::shared_ptr<sc2430_impl>;
    sc2430_impl(uhd::rfnoc::radio_control::sptr radio, uhd::property_tree::sptr tree);
    ~sc2430_impl() = default;

    double get_tx_frequency(const size_t chan) override
    {
        return _radio->get_tx_frequency(chan);
    }

    double get_rx_frequency(const size_t chan) override
    {
        return _radio->get_rx_frequency(chan);
    }

    double set_tx_frequency(const double freq, size_t chan) override
    {
        auto& prop =
            _tree->access<double>(uhd::fs_path(scm::NAME)
                                  / get_frontend_path(chan, uhd::TX_DIRECTION) / "freq" / "value");
        prop.set(freq);
        return prop.get();
    }

    double set_rx_frequency(const double freq, const size_t chan) override
    {
        auto& prop =
            _tree->access<double>(uhd::fs_path(scm::NAME)
                                  / get_frontend_path(chan, uhd::RX_DIRECTION) / "freq" / "value");
        prop.set(freq);
        return prop.get();
    }

    void set_tx_tune_args(const uhd::device_addr_t& args, const size_t chan) override
    {
        const auto path =
            uhd::fs_path(scm::NAME) / get_frontend_path(chan, uhd::TX_DIRECTION) / "band" / "value";
        auto& prop = _tree->access<std::string>(path);
        prop.set(args.get("band", ""));
    }

    void set_rx_tune_args(const uhd::device_addr_t& args, const size_t chan) override
    {
        const auto path =
            uhd::fs_path(scm::NAME) / get_frontend_path(chan, uhd::RX_DIRECTION) / "band" / "value";
        auto& prop = _tree->access<std::string>(path);
        prop.set(args.get("band", ""));
    }

    std::vector<std::string> get_tx_gain_names(const size_t) const override
    {
        return {"all"};
    }

    std::vector<std::string> get_rx_gain_names(const size_t) const override
    {
        return {"all"};
    }

    double get_tx_gain(const std::string& name_, const size_t chan) override
    {
        const std::string name        = name_.empty() ? "all" : name_;
        const uhd::fs_path gains_path = get_frontend_path(chan, uhd::TX_DIRECTION) / "gains";
        return _tree->access<double>(gains_path / name / "value").get();
    }

    double get_rx_gain(const std::string& name_, const size_t chan) override
    {
        const std::string name        = name_.empty() ? "all" : name_;
        const uhd::fs_path gains_path = get_frontend_path(chan, uhd::RX_DIRECTION) / "gains";
        return _tree->access<double>(gains_path / name / "value").get();
    }

    double set_tx_gain(const double gain, const std::string& name_, const size_t chan) override
    {
        const std::string name        = name_.empty() ? "all" : name_;
        const uhd::fs_path gains_path = get_frontend_path(chan, uhd::TX_DIRECTION) / "gains";
        return _tree->access<double>(gains_path / name / "value").set(gain).get();
    }

    double set_rx_gain(const double gain, const std::string& name_, const size_t chan) override
    {
        const std::string name        = name_.empty() ? "all" : name_;
        const uhd::fs_path gains_path = get_frontend_path(chan, uhd::RX_DIRECTION) / "gains";
        return _tree->access<double>(gains_path / name / "value").set(gain).get();
    }

    void set_rx_agc(const bool, const size_t) override
    {
        throw uhd::not_implemented_error("set_rx_agc is not supported on this radio");
    }

    uhd::meta_range_t get_tx_bandwidth_range(const size_t chan) const override
    {
        return _radio->get_tx_bandwidth_range(chan);
    }

    uhd::meta_range_t get_rx_bandwidth_range(const size_t chan) const override
    {
        return _radio->get_rx_bandwidth_range(chan);
    }

    double get_tx_bandwidth(const size_t chan) override
    {
        return _radio->get_tx_bandwidth(chan);
    }

    double get_rx_bandwidth(const size_t chan) override
    {
        return _radio->get_rx_bandwidth(chan);
    }

    double set_tx_bandwidth(const double bandwidth, const size_t chan) override
    {
        return _radio->set_tx_bandwidth(bandwidth, chan);
    }

    double set_rx_bandwidth(const double bandwidth, const size_t chan) override
    {
        return _radio->set_rx_bandwidth(bandwidth, chan);
    }

    std::vector<std::string> get_tx_lo_names(const size_t) const override
    {
        return {};
    }

    std::vector<std::string> get_rx_lo_names(const size_t) const override
    {
        return {};
    }

    std::vector<std::string> get_tx_lo_sources(const std::string&, const size_t) const override
    {
        return {};
    }

    std::vector<std::string> get_rx_lo_sources(const std::string&, const size_t) const override
    {
        return {};
    }

    uhd::freq_range_t get_tx_lo_freq_range(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_tx_lo_freq_range is not supported on this radio");
    }

    uhd::freq_range_t get_rx_lo_freq_range(const std::string&, const size_t) const override
    {
        throw uhd::not_implemented_error("get_rx_lo_freq_range is not supported on this radio");
    }

    void set_rx_lo_source(const std::string&, const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("set_rx_lo_source is not supported on this radio");
    }

    void set_tx_lo_source(const std::string&, const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("set_tx_lo_source is not supported on this radio");
    }

    const std::string get_rx_lo_source(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_rx_lo_source is not supported on this radio");
    }

    const std::string get_tx_lo_source(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_tx_lo_source is not supported on this radio");
    }

    void set_rx_lo_export_enabled(bool, const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("set_rx_lo_export_enabled is not supported on this radio");
    }

    void set_tx_lo_export_enabled(const bool, const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("set_tx_lo_export_enabled is not supported on this radio");
    }

    bool get_rx_lo_export_enabled(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_rx_lo_export_enabled is not supported on this radio");
    }

    bool get_tx_lo_export_enabled(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_tx_lo_export_enabled is not supported on this radio");
    }

    double set_rx_lo_freq(double, const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("set_rx_lo_freq is not supported on this radio");
    }

    double set_tx_lo_freq(const double, const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("set_tx_lo_freq is not supported on this radio");
    }

    double get_rx_lo_freq(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_rx_lo_freq is not supported on this radio");
    }

    double get_tx_lo_freq(const std::string&, const size_t) override
    {
        throw uhd::not_implemented_error("get_tx_lo_freq is not supported on this radio");
    }

    bool has_rx_power_reference(const size_t chan) override
    {
        return _radio->has_rx_power_reference(chan);
    }

    bool has_tx_power_reference(const size_t chan) override
    {
        return _radio->has_tx_power_reference(chan);
    }

    void set_rx_power_reference(const double power_dbm, const size_t chan) override
    {
        const uhd::fs_path power_path = get_frontend_path(chan, uhd::RX_DIRECTION) / "power_ref";
        _tree->access<double>(power_path / "value").set(power_dbm);
    }

    void set_tx_power_reference(const double power_dbm, const size_t chan) override
    {
        const uhd::fs_path power_path = get_frontend_path(chan, uhd::TX_DIRECTION) / "power_ref/";
        _tree->access<double>(power_path / "value").set(power_dbm);
    }

    double get_rx_power_reference(const size_t chan) override
    {
        const uhd::fs_path power_path = get_frontend_path(chan, uhd::RX_DIRECTION) / "power_ref";
        return _tree->access<double>(power_path / "value").get();
    }

    double get_tx_power_reference(const size_t chan) override
    {
        const uhd::fs_path power_path = get_frontend_path(chan, uhd::TX_DIRECTION) / "power_ref/";
        return _tree->access<double>(power_path / "value").get();
    }

    std::vector<std::string> get_rx_power_ref_keys(const size_t) override
    {
        throw uhd::not_implemented_error("get_rx_power_ref_keys is not supported on this radio");
    }

    std::vector<std::string> get_tx_power_ref_keys(const size_t) override
    {
        throw uhd::not_implemented_error("get_tx_power_ref_keys is not supported on this radio");
    }

    uhd::meta_range_t get_rx_power_range(const size_t) override
    {
        throw uhd::not_implemented_error("get_rx_power_range is not supported on this radio");
    }

    uhd::meta_range_t get_tx_power_range(const size_t) override
    {
        throw uhd::not_implemented_error("get_tx_power_range is not supported on this radio");
    }

    uhd::gain_range_t get_rx_gain_range(const std::string& name, const size_t chan) const override;
    uhd::gain_range_t get_tx_gain_range(const std::string& name, const size_t chan) const override;

    uhd::freq_range_t get_rx_frequency_range(const size_t) const override;
    uhd::freq_range_t get_tx_frequency_range(const size_t) const override;

    void set_gain_updates(const bool enabled, const size_t chan) override;
    bool get_gain_updates(const size_t chan) override;

    uint16_t read_reg(const uint8_t address) override;
    void write_reg(const uint8_t address, const uint16_t value) override;

    std::string get_name() override
    {
        return scm::NAME;
    }

private:
    uhd::fs_path get_frontend_path(const size_t chan, const uhd::direction_t trx) const
    {
        uhd::fs_path frontend = (trx == uhd::TX_DIRECTION) ? "tx_frontends" : "rx_frontends";
        return frontend / chan;
    }

    void initialize_path(const size_t chan, const uhd::direction_t trx);
};

} // namespace scm
