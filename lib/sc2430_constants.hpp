//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/usrp/zbx_tune_map_item.hpp>

namespace scm {

// Extension Name
static constexpr char NAME[] = "sc2430";

// Number of Channels per SCM Slot
static constexpr size_t CHANNELS_PER_SLOT = 2;

// Default power reference settings resulting in minimum gain/power levels.
static constexpr double RX_DEFAULT_POWER_REF = 10;
static constexpr double TX_DEFAULT_POWER_REF = -60;

// The ZBX is capable of TX output power levels exceeding the SCM TX input no-damage
// specification of +15 dBm. For good measure, back this off by another 5 dB.
static constexpr double TX_IN_ABS_MAX_POWER = 10;
// The X410 specification indicates the TX EVM is optimized between -15 and 5 dBm for a 5G
// NR 100 MHz waveform. Limiting the ZBX output power to -5 dBm provides a margin for other
// waveform types. The SCM has sufficient TX gain to increase the ZBX output level up to
// the SCM TX output power specification (Linear Power assuming 10 dB PAR).
static constexpr double TX_IN_TYP_MAX_POWER = -5;

} // namespace scm

// TX Tune Map for the ZBX
namespace uhd { namespace usrp { namespace zbx { namespace scm {
// clang-format off
const std::vector<uhd::usrp::zbx::zbx_tune_map_item_t> ZBX_TX_TUNE_MAP = {
//  | min_band_freq | max_band_freq | rf_fltr | if1_fltr | if2_fltr | lo1_inj_side | lo2_inj_side | if1_freq_min | if1_freq_max | if2_freq_min | if2_freq_max |
    {      1e6,          200e6,           1,        2,         1,        HIGH,            LOW,       4600e6,        4600e6,        1060e6,        1060e6    },
    {    200e6,          300e6,           1,        2,         1,        HIGH,            LOW,       4600e6,        4600e6,        1060e6,        1060e6    },
    {    300e6,          400e6,           1,        2,         1,        HIGH,            LOW,       4600e6,        4600e6,        1060e6,        1060e6    },
    {    400e6,          600e6,           1,        2,         1,        HIGH,            LOW,       4600e6,        4600e6,        1060e6,        1060e6    },
    {    600e6,          800e6,           1,        2,         1,        HIGH,            LOW,       4600e6,        4600e6,        1060e6,        1060e6    },
    {    800e6,         1300e6,           1,        2,         1,        HIGH,            LOW,       4600e6,        4600e6,        1060e6,        1060e6    },
    {   1300e6,         1800e6,           1,        2,         1,        HIGH,            LOW,       4600e6,        4600e6,        1060e6,        1060e6    },
    {   1800e6,         2300e6,           2,        1,         1,        HIGH,           HIGH,       4100e6,        4100e6,        1060e6,        1060e6    },
    {   2300e6,         2700e6,           3,        1,         2,        HIGH,           HIGH,       3700e6,        3700e6,        2070e6,        2200e6    },
    {   2700e6,         3000e6,           3,        5,         2,         LOW,            LOW,       6800e6,        7100e6,        2000e6,        2000e6    },
    // Keep the same LO2 frequency (6.0928e9) from 3.8G to 4.03G
    {   3000e6,         3800e6,           0,        1,         2,        NONE,           HIGH,            0,             0,        2050e6,        2298.5e6  },
    {   3800e6,         4030e6,           0,        1,         2,        NONE,           HIGH,            0,             0,        2298.5e6,      2068.5e6  },
    {   4030e6,         4500e6,           0,        1,         1,        NONE,           HIGH,            0,             0,        1060e6,        1060e6    },
    // Keep the same LO2 frequency from 4.8G to 5.1G
    {   4500e6,         4800e6,           0,        2,         1,        NONE,            LOW,            0,             0,        1060e6,        1060e6    },
    {   4800e6,         5100e6,           0,        2,         1,        NONE,            LOW,            0,             0,        1060e6,        1360e6    },
    // Keep the same LO2 frequency from 5.35G down to 5.15G
    {   5100e6,         5150e6,           0,        3,         2,        NONE,            LOW,            0,             0,        1900e6,        1944.88e6 },
    {   5150e6,         5350e6,           0,        3,         2,        NONE,            LOW,            0,             0,        1873.2e6,      2073.2e6  },
    {   5350e6,         5700e6,           0,        3,         2,        NONE,            LOW,            0,             0,        2073.2e6,      2300e6    },
    // Keep the same LO2 frequency from 6.125G down to 5.925G
    {   5700e6,         5925e6,           0,        4,         2,        NONE,            LOW,            0,             0,        2300e6,        2422.92e6 },
    {   5925e6,         6125e6,           0,        4,         2,        NONE,            LOW,            0,             0,        2218.12e6,     2418.12e6 },
    {   6125e6,         6400e6,           0,        4,         2,        NONE,            LOW,            0,             0,        2418.12e6,     2500e6    },
    // Keep the same LO2 frequency from 6.925G up to 7.125G
    {   6400e6,         6925e6,           0,        5,         2,        NONE,            LOW,            0,             0,        1900e6,        1943.75e6 },
    {   6925e6,         7125e6,           0,        5,         2,        NONE,            LOW,            0,             0,        1943.75e6,     2143.75e6 },
    {   7125e6,         7400e6,           0,        6,         1,        NONE,            LOW,            0,             0,        1060e6,        1060e6    },
    {   7400e6,         8000e6,           0,        6,         2,        NONE,            LOW,            0,             0,        1950e6,        2050e6    },
};
// clang-format on
}}}} // namespace uhd::usrp::zbx::scm
