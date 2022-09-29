//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/types/ranges.hpp>
#include <string>

namespace scm {

// Represents the programming specification for the SC2430 filters.
//
// SC2430's filters are divided into multiple bands, each of which has 16 evenly spaced
// sub-frequencies. The band id selects which filter to use, and the frequency index is
// used within the SC2430 to determine the proper gain calibration coefficients.
//
// The coerced_frequency is the closest sub-frequency available.
struct filter_spec
{
    size_t band_id;
    size_t frequency_idx;
    double coerced_frequency;

    filter_spec(const std::string& band_name, double frequency);
    filter_spec() = delete;
};

// Get the combined frequency range of all filters
uhd::freq_range_t get_frequency_range(void);
// Get the default band for the frequency specified
std::string get_default_band(double frequency);

} // namespace scm
