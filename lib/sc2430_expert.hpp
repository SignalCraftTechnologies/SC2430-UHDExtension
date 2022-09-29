//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include "sc2430_phy.hpp"
#include <uhd/experts/expert_nodes.hpp>

namespace scm {

// This expert is responsible for programming frequency and filter bands
class frequency_programming_expert : public uhd::experts::worker_node_t
{
public:
    frequency_programming_expert(const uhd::experts::node_retriever_t& db,
        uhd::rfnoc::radio_control::sptr radio,
        phy::sptr phy,
        const path& path,
        const uhd::fs_path scm_fe_path)
        : uhd::experts::worker_node_t(scm_fe_path / "scm_freq_programming_expert")
        , _radio(radio)
        , _phy(phy)
        , _path(path)
        , _scm_frequency_in(db, scm_fe_path / "freq" / "value" / "desired")
        , _band_select_in(db, scm_fe_path / "band" / "value" / "desired")
        , _scm_frequency_out(db, scm_fe_path / "freq" / "value" / "coerced")
        , _band_select_out(db, scm_fe_path / "band" / "value" / "coerced")
    {
        bind_accessor(_band_select_in);
        bind_accessor(_scm_frequency_in);
        bind_accessor(_scm_frequency_out);
        bind_accessor(_band_select_out);
    }

private:
    void resolve(void) override;

    uhd::rfnoc::radio_control::sptr _radio;
    phy::sptr _phy;
    const path _path;

    // Inputs
    uhd::experts::data_reader_t<double> _scm_frequency_in;
    uhd::experts::data_reader_t<std::string> _band_select_in;
    // Outputs
    uhd::experts::data_writer_t<double> _scm_frequency_out;
    uhd::experts::data_writer_t<std::string> _band_select_out;
};

// This expert is responsible for programming the gain settings
class gain_programming_expert : public uhd::experts::worker_node_t
{
public:
    gain_programming_expert(const uhd::experts::node_retriever_t& db,
        phy::sptr phy,
        const path& path,
        const uhd::fs_path scm_fe_path)
        : uhd::experts::worker_node_t(scm_fe_path / "scm_gain_expert")
        , _phy(phy)
        , _path(path)
        , _scm_gain_in(db, scm_fe_path / "gains" / "all" / "value" / "desired")
        , _scm_gain_out(db, scm_fe_path / "gains" / "all" / "value" / "coerced")
    {
        bind_accessor(_scm_gain_in);
        bind_accessor(_scm_gain_out);
    }

private:
    void resolve(void) override;

    phy::sptr _phy;
    const path _path;

    // Inputs
    uhd::experts::data_reader_t<double> _scm_gain_in;
    // Outputs
    uhd::experts::data_writer_t<double> _scm_gain_out;
};

// This expert is responsible for distributing the gain between the ZBX and SCM
class gain_expert : public uhd::experts::worker_node_t
{
public:
    gain_expert(const uhd::experts::node_retriever_t& db,
        uhd::rfnoc::radio_control::sptr radio,
        phy::sptr phy,
        const path& path,
        const uhd::fs_path scm_fe_path,
        const uhd::fs_path fe_path)
        : uhd::experts::worker_node_t(scm_fe_path / "zbx_gain_expert")
        , _radio(radio)
        , _phy(phy)
        , _path(path)
        , _power_ref_path(fe_path / "power_ref" / "enabled")
        , _gain_in(db, fe_path / "gains" / "all" / "value" / "desired")
        , _gain_out(db, fe_path / "gains" / "all" / "value" / "coerced")
        , _scm_gain_out(db, scm_fe_path / "gains" / "all" / "value" / "desired")
    {
        bind_accessor(_gain_in);
        bind_accessor(_gain_out);
        bind_accessor(_scm_gain_out);
    }

private:
    void resolve(void) override;

    uhd::rfnoc::radio_control::sptr _radio;
    phy::sptr _phy;
    const path _path;
    const uhd::fs_path _power_ref_path;

    // Inputs
    uhd::experts::data_reader_t<double> _gain_in;
    // Outputs
    uhd::experts::data_writer_t<double> _gain_out;
    uhd::experts::data_writer_t<double> _scm_gain_out;
};

// This expert is responsible for setting the power reference with respect to the SCM ANT port
class power_ref_expert : public uhd::experts::worker_node_t
{
public:
    power_ref_expert(const uhd::experts::node_retriever_t& db,
        uhd::rfnoc::radio_control::sptr radio,
        phy::sptr phy,
        const path& path,
        const uhd::fs_path scm_fe_path,
        const uhd::fs_path fe_path)
        : uhd::experts::worker_node_t(scm_fe_path / "zbx_power_ref_expert")
        , _radio(radio)
        , _phy(phy)
        , _path(path)
        , _power_ref_path(fe_path / "power_ref" / "enabled")
        , _power_ref_in(db, fe_path / "power_ref" / "value" / "desired")
        , _freq_in(db, scm_fe_path / "freq" / "value" / "coerced")
        , _power_ref_out(db, fe_path / "power_ref" / "value" / "coerced")
        , _scm_gain_out(db, scm_fe_path / "gains" / "all" / "value" / "desired")
    {
        bind_accessor(_power_ref_in);
        bind_accessor(_freq_in);
        bind_accessor(_power_ref_out);
        bind_accessor(_scm_gain_out);
    }

private:
    void resolve(void) override;

    uhd::rfnoc::radio_control::sptr _radio;
    phy::sptr _phy;
    const path _path;
    const uhd::fs_path _power_ref_path;

    // Inputs
    uhd::experts::data_reader_t<double> _power_ref_in;
    uhd::experts::data_reader_t<double> _freq_in;
    // Outputs
    uhd::experts::data_writer_t<double> _power_ref_out;
    uhd::experts::data_writer_t<double> _scm_gain_out;
};

} // namespace scm
