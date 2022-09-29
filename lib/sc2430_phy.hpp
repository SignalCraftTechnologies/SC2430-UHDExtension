//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include "version.hpp"
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/types/serial.hpp>
#include <array>
#include <map>

namespace scm {

struct filter_spec;

struct path
{
    size_t chan;
    uhd::direction_t trx;
};

// This class contains the functionality to control the SC2430 via SPI and GPIO
class phy final
{
private:
    uhd::rfnoc::radio_control::sptr _radio;
    uhd::spi_iface::sptr _spi;
    std::array<size_t, 2> _dividers;
    std::map<std::pair<std::string, size_t>, uhd::gain_range_t> _tx_gain_range;
    std::map<std::pair<std::string, size_t>, uhd::gain_range_t> _rx_gain_range;
    version _fw_version;
    version _hw_version;

public:
    using sptr = std::shared_ptr<phy>;
    phy(uhd::rfnoc::radio_control::sptr radio);

    uint32_t read_general(const uint8_t address);
    uint32_t write_general(const uint8_t address, const uint16_t value);

    void set_attn_latch(const size_t chan, const bool enabled);
    bool get_attn_latch(const size_t chan);
    void set_filter_spec(const path& path, const filter_spec& filter);
    void set_gain(const path& path, const int8_t gain);
    void set_tx_pa(const size_t chan, const bool enable);

    uhd::gain_range_t get_tx_gain_range(const size_t chan) const;
    uhd::gain_range_t get_rx_gain_range(const size_t chan) const;

private:
    struct control;
    void write_control(const control& control);

    void validate_spi(void);
    void set_triggers(const path& path, const uint8_t value);
    void update_gain_range(const path& path);

    size_t clock_to_divider(const double radio_clk, double spi_clk) const;
    uint32_t get_gpio_shift(void) const;
};
} // namespace scm
