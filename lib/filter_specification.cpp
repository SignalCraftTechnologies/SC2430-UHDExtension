//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "filter_specification.hpp"
#include <uhd/exception.hpp>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <algorithm>
#include <map>
#include <vector>

namespace {

// Control Protocol Filter Values
enum band_index {
    FILTER_BAND_n39     = 0x00,
    FILTER_BAND_n34     = 0x01,
    FILTER_BAND_n40     = 0x02,
    FILTER_BAND_n41     = 0x03,
    FILTER_BAND_n38     = 0x04,
    FILTER_BAND_n78     = 0x05,
    FILTER_BAND_n48     = 0x06,
    FILTER_BAND_n77     = 0x07,
    FILTER_BAND_n79     = 0x08,
    FILTER_BAND_n46_n47 = 0x09,
    FILTER_BAND_n96     = 0x0A,
    FILTER_BAND_bypass  = 0x0B,
};

struct frequency_band
{
    band_index id;
    double lower;
    double upper;

    double midpoint() const
    {
        return (upper - lower) / 2.0 + lower;
    }

    size_t to_frequency_index(double frequency) const
    {
        const size_t min_index = 0;
        const size_t max_index = 15;

        double position = (frequency - lower) / (upper - lower) * max_index;
        size_t index    = round(position);
        index           = (index < min_index) ? min_index : index;
        index           = (index > max_index) ? max_index : index;
        return index;
    }
};

// clang-format off
static const std::map<std::string, const frequency_band> frequency_bands = {
    {"n39",     {FILTER_BAND_n39,       1880e6, 1920e6}},
    {"n34",     {FILTER_BAND_n34,       2010e6, 2025e6}},
    {"n40",     {FILTER_BAND_n40,       2300e6, 2400e6}},
    {"n41",     {FILTER_BAND_n41,       2496e6, 2690e6}},
    {"n38",     {FILTER_BAND_n38,       2570e6, 2620e6}},
    {"n78",     {FILTER_BAND_n78,       3300e6, 3800e6}},
    {"n48",     {FILTER_BAND_n48,       3550e6, 3700e6}},
    {"n77",     {FILTER_BAND_n77,       3300e6, 4200e6}},
    {"n79",     {FILTER_BAND_n79,       4400e6, 5000e6}},
    {"n46/n47", {FILTER_BAND_n46_n47,   5150e6, 5925e6}},
    {"n96",     {FILTER_BAND_n96,       5925e6, 7125e6}},
    {"bypass",  {FILTER_BAND_bypass,    350e6,  7200e6}}
};
// clang-format on

} // namespace

namespace scm {

filter_spec::filter_spec(const std::string& band_name, double frequency)
{
    const auto& band_pair = frequency_bands.find(band_name);
    if (band_pair == frequency_bands.end()) {
        throw uhd::value_error(std::string("Unrecognized band ") + band_name);
    }

    const auto& band = band_pair->second;

    frequency = (frequency < band.lower) ? band.lower : frequency;
    frequency = (frequency > band.upper) ? band.upper : frequency;

    this->band_id           = band.id;
    this->frequency_idx     = band.to_frequency_index(frequency);
    this->coerced_frequency = frequency;
}

uhd::freq_range_t get_frequency_range(void)
{
    std::vector<uhd::range_t> ranges;
    for (const auto& band : frequency_bands)
        ranges.emplace_back(band.second.lower, band.second.upper);
    return (uhd::freq_range_t(ranges.begin(), ranges.end())).as_monotonic();
}

std::string get_default_band(const double frequency)
{
    // Find all filtered bands that could apply
    std::vector<std::string> bands;
    for (const auto& band : frequency_bands) {
        if (band.first == "bypass")
            continue;
        if (band.second.lower <= frequency && frequency <= band.second.upper)
            bands.push_back(band.first);
    }

    // If it's empty use the bypass path
    if (bands.empty()) {
        bands.push_back("bypass");
    } else {
        // Select the filtered band with the closest center frequency
        std::sort(bands.begin(),
            bands.end(),
            [frequency](const std::string& band1, const std::string& band2) {
                const double center1 = frequency_bands.at(band1).midpoint();
                const double center2 = frequency_bands.at(band2).midpoint();
                return fabs(center1 - frequency) < fabs(center2 - frequency);
            });
    }
    return bands[0];
}

} // namespace scm
