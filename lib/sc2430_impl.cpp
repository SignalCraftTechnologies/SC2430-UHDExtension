//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "sc2430_impl.hpp"
#include "filter_specification.hpp"
#include "sc2430_constants.hpp"
#include "sc2430_expert.hpp"
#include "sc2430_phy.hpp"
#include <uhd/experts/expert_container.hpp>
#include <uhd/extension/extension.hpp>
#include <uhd/rfnoc/registry.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/math.hpp>
#include <cassert>

using namespace uhd;

namespace {

const std::vector<std::string> ANTENNAS = {"ANT"};

const std::unordered_map<std::string, std::string> ANTENNA_NAME_COMPAT_MAP = {};

} // namespace

namespace scm {

sc2430::sptr sc2430::make(uhd::extension::extension::factory_args fargs)
{
    auto tree              = fargs.radio_ctrl->get_tree();
    const auto is_valid_db = tree->exists("dboard/rx_frontends/0/name")
                             && tree->access<std::string>("dboard/rx_frontends/0/name").get()
                                    == "ZBX";
    if (!is_valid_db)
        throw uhd::runtime_error("Daughterboard for " + std::string(scm::NAME) + " not found.");

    return std::make_shared<sc2430_impl>(fargs.radio_ctrl, fargs.radio_ctrl->get_tree());
}

sc2430_impl::sc2430_impl(uhd::rfnoc::radio_control::sptr radio, uhd::property_tree::sptr tree)
    : nameless_gain_mixin([](uhd::direction_t, size_t) { return "all"; })
    , _radio(radio)
    , _tree(tree)
    , _phy(std::make_shared<phy>(radio))
{
    for (size_t chan = 0; chan < scm::CHANNELS_PER_SLOT; chan++) {
        _radio->set_tx_antenna("TX/RX0", chan);
        _radio->set_rx_antenna("RX1", chan);
    }

    _rx_antenna = std::make_shared<uhd::rfnoc::rf_control::enumerated_antenna>(
        tree,
        [this](size_t chan) {
            return this->get_frontend_path(chan, RX_DIRECTION) / "antenna" / "value";
        },
        ANTENNAS,
        ANTENNA_NAME_COMPAT_MAP);
    _tx_antenna = std::make_shared<uhd::rfnoc::rf_control::enumerated_antenna>(
        tree,
        [this](size_t chan) {
            return this->get_frontend_path(chan, TX_DIRECTION) / "antenna" / "value";
        },
        ANTENNAS,
        ANTENNA_NAME_COMPAT_MAP);

    _expert_container = uhd::experts::expert_factory::create_container("sc2430_radio");

    for (size_t chan = 0; chan < scm::CHANNELS_PER_SLOT; chan++) {
        initialize_path(chan, TX_DIRECTION);
        initialize_path(chan, RX_DIRECTION);
    }
    _expert_container->resolve_all();
}

void sc2430_impl::initialize_path(const size_t chan, const uhd::direction_t trx)
{
    const path path           = {chan, trx};
    const auto fe_path        = get_frontend_path(chan, trx);
    const auto scm_fe_path    = fs_path(scm::NAME) / fe_path;
    const auto dboard_fe_path = fs_path("dboard") / fe_path;

    // Antenna
    _tree->create<std::vector<std::string>>(fe_path / "antenna" / "options")
        .set(trx == TX_DIRECTION ? get_tx_antennas(chan) : get_rx_antennas(chan))
        .add_coerced_subscriber([](const std::vector<std::string>&) {
            throw uhd::runtime_error("Attempting to update antenna options!");
        });
    _tree->create<std::string>(fe_path / "antenna" / "value")
        .set(ANTENNAS[0])
        .set_coercer([](const std::string& antenna) {
            UHD_ASSERT_THROW(ANTENNAS.size() == 1);
            if (antenna != ANTENNAS[0])
                throw uhd::runtime_error("Set antenna to invalid value!");
            return antenna;
        });

    // Frequency Related
    uhd::experts::expert_factory::add_dual_prop_node<double>(_expert_container,
        _tree,
        scm_fe_path / "freq" / "value",
        0,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_dual_prop_node<std::string>(_expert_container,
        _tree,
        scm_fe_path / "band" / "value",
        "",
        uhd::experts::AUTO_RESOLVE_OFF);


    uhd::experts::expert_factory::add_worker_node<frequency_programming_expert>(
        _expert_container, _expert_container->node_retriever(), _radio, _phy, path, scm_fe_path);

    if (trx == TX_DIRECTION) {
        _tree
            ->access<std::vector<uhd::usrp::zbx::zbx_tune_map_item_t>>(
                dboard_fe_path / "tune_table")
            .set(uhd::usrp::zbx::scm::ZBX_TX_TUNE_MAP);
    }

    // Gain Related
    uhd::experts::expert_factory::add_dual_prop_node<double>(_expert_container,
        _tree,
        scm_fe_path / "gains" / "all" / "value",
        trx == TX_DIRECTION ? _phy->get_tx_gain_range(chan).start()
                            : _phy->get_rx_gain_range(chan).start(),
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_dual_prop_node<double>(_expert_container,
        _tree,
        fe_path / "gains" / "all" / "value",
        trx == TX_DIRECTION ? _phy->get_tx_gain_range(chan).start()
                            : _phy->get_rx_gain_range(chan).start(),
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_worker_node<gain_expert>(_expert_container,
        _expert_container->node_retriever(),
        _radio,
        _phy,
        path,
        scm_fe_path,
        fe_path);

    uhd::experts::expert_factory::add_worker_node<gain_programming_expert>(
        _expert_container, _expert_container->node_retriever(), _phy, path, scm_fe_path);

    // Power Reference Related
    _tree->create<bool>(fe_path / "power_ref" / "enabled").set(true);

    uhd::experts::expert_factory::add_dual_prop_node<double>(_expert_container,
        _tree,
        fe_path / "power_ref" / "value",
        trx == TX_DIRECTION ? TX_DEFAULT_POWER_REF : RX_DEFAULT_POWER_REF,
        uhd::experts::AUTO_RESOLVE_ON_WRITE);

    uhd::experts::expert_factory::add_worker_node<power_ref_expert>(_expert_container,
        _expert_container->node_retriever(),
        _radio,
        _phy,
        path,
        scm_fe_path,
        fe_path);
}

uhd::gain_range_t sc2430_impl::get_rx_gain_range(const std::string& name, const size_t chan) const
{
    const auto scm_range = _phy->get_rx_gain_range(chan);
    const auto zbx_range = _radio->get_rx_gain_range(name, chan);
    return {zbx_range.start() + scm_range.start(), zbx_range.stop() + scm_range.stop(), 1};
}

uhd::gain_range_t sc2430_impl::get_tx_gain_range(const std::string& name, const size_t chan) const
{
    const auto scm_range = _phy->get_tx_gain_range(chan);
    const auto zbx_range = _radio->get_tx_gain_range(name, chan);
    return {zbx_range.start() + scm_range.start(), zbx_range.stop() + scm_range.stop(), 1};
}

freq_range_t sc2430_impl::get_rx_frequency_range(const size_t) const
{
    return get_frequency_range();
}

freq_range_t sc2430_impl::get_tx_frequency_range(const size_t) const
{
    return get_frequency_range();
}

void sc2430_impl::set_gain_updates(const bool enabled, const size_t chan)
{
    return _phy->set_attn_latch(chan, enabled);
}

bool sc2430_impl::get_gain_updates(const size_t chan)
{
    return _phy->get_attn_latch(chan);
}

uint16_t sc2430_impl::read_reg(const uint8_t address)
{
    return _phy->read_general(address);
}

void sc2430_impl::write_reg(const uint8_t address, const uint16_t value)
{
    _phy->write_general(address, value);
}

} // namespace scm

/*!
 * Register the SC2430 extension in the extension registry of UHD
 */
UHD_REGISTER_EXTENSION(sc2430, scm::sc2430);
