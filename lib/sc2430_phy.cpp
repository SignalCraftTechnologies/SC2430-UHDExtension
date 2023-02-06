//
// Copyright 2022 SignalCraft Technologies Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "sc2430_phy.hpp"
#include "filter_specification.hpp"
#include "sc2430_constants.hpp"
#include <uhd/features/spi_getter_iface.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/utils/log.hpp>

namespace {

// GPIO Pinouts
// https://uhd.readthedocs.io/en/latest/page_x400_gpio_api.html
constexpr uint32_t GPIO_PCLK            = 0; // HDMI Pin 1, Data[0]
constexpr uint32_t GPIO_PCLK_MASK       = 1 << GPIO_PCLK;
constexpr uint32_t GPIO_RESERVED_1      = 1; // HDMI Pin 3, Data[1]
constexpr uint32_t GPIO_RESERVED_1_MASK = 1 << GPIO_RESERVED_1;
constexpr uint32_t GPIO_SDI             = 2; // HDMI Pin 4, Data[2]
constexpr uint32_t GPIO_SDI_MASK        = 1 << GPIO_SDI;
constexpr uint32_t GPIO_SDO             = 3; // HDMI Pin 6, Data[3]
constexpr uint32_t GPIO_SDO_MASK        = 1 << GPIO_SDO;
constexpr uint32_t GPIO_CONTROL_CS      = 4; // HDMI Pin 7, Data[4]
constexpr uint32_t GPIO_CONTROL_CS_MASK = 1 << GPIO_CONTROL_CS;
constexpr uint32_t GPIO_GENERAL_CS      = 5; // HDMI Pin 9, Data[5]
constexpr uint32_t GPIO_GENERAL_CS_MASK = 1 << GPIO_GENERAL_CS;
constexpr uint32_t GPIO_ATR0_CH0        = 6; // HDMI Pin 10, Data[6]
constexpr uint32_t GPIO_ATR0_MASK       = 1 << GPIO_ATR0_CH0;
constexpr uint32_t GPIO_ATR1_CH1        = 7; // HDMI Pin 12, Data[7]
constexpr uint32_t GPIO_ATR1_MASK       = 1 << GPIO_ATR1_CH1;
constexpr uint32_t GPIO_LATCH_CH0       = 8; // HDMI Pin 13, Data[8]
constexpr uint32_t GPIO_LATCH_CH0_MASK  = 1 << GPIO_LATCH_CH0;
constexpr uint32_t GPIO_LATCH_CH1       = 9; // HDMI Pin 15, Data[9]
constexpr uint32_t GPIO_LATCH_CH1_MASK  = 1 << GPIO_LATCH_CH1;
constexpr uint32_t GPIO_RESERVED_2      = 10; // HDMI Pin 16, Data[10]
constexpr uint32_t GPIO_RESERVED_2_MASK = 1 << GPIO_RESERVED_2;
constexpr uint32_t GPIO_SPI_RESET       = 11; // HDMI Pin 19, Data[11]
constexpr uint32_t GPIO_SPI_RESET_MASK  = 1 << GPIO_SPI_RESET;
constexpr uint32_t GPIO_OUTPUT_MASK =
    GPIO_PCLK_MASK | GPIO_SDO_MASK | GPIO_CONTROL_CS_MASK | GPIO_GENERAL_CS_MASK | GPIO_ATR0_MASK
    | GPIO_ATR1_MASK | GPIO_LATCH_CH0_MASK | GPIO_LATCH_CH1_MASK | GPIO_SPI_RESET_MASK;

// SPI Interface Index
constexpr size_t GENERAL_SPI = 0;
constexpr size_t CONTROL_SPI = 1;

// General Protocol Registers
constexpr uint8_t SLOT_DETECT_REG       = 0x00;
constexpr uint8_t PRODUCT_ID_REG        = 0x01;
constexpr uint8_t HW_VER_REG            = 0x02;
constexpr uint8_t FW_VER_REG            = 0x03;
constexpr uint8_t SCRATCH_REG           = 0x07;
constexpr uint8_t GAIN_RANGE_RX_CH0_REG = 0x0E;
constexpr uint8_t GAIN_RANGE_TX_CH0_REG = 0x0F;
constexpr uint8_t GAIN_RANGE_RX_CH1_REG = 0x10;
constexpr uint8_t GAIN_RANGE_TX_CH1_REG = 0x11;

// Control ID
enum class control_type { ATTEN = 0b00, GAIN = 0b01, SWTRIG = 0b10, FILTER = 0b11 };

// Element ID
constexpr uint8_t GAIN_TX_GAIN_CTRL_ID     = 0x0;
constexpr uint8_t GAIN_RX_GAIN_CTRL_ID     = 0x0;
constexpr uint8_t GAIN_TX_PA_CTRL_ID       = 0x1;
constexpr uint8_t SWTRIG_LATCH_SRC_CTRL_ID = 0x0;
constexpr uint8_t SWTRIG_DIR_SRC_CTRL_ID   = 0x1;
constexpr uint8_t FILTER_TX_BAND_CTRL_ID   = 0x2;
constexpr uint8_t FILTER_RX_BAND_CTRL_ID   = 0x1;
// Element Values
constexpr uint8_t SWTRIG_SRC_ELEMENT_VAL = 0x00;
constexpr uint8_t SWTRIG_SRC_GPIO_VAL    = 0x01;

// Nominal Gain Ranges
constexpr double TX_NOMINAL_MIN_GAIN = -39;
constexpr double TX_NOMINAL_MAX_GAIN = 24;
uhd::gain_range_t TX_NOMINAL_GAIN_RANGE{TX_NOMINAL_MIN_GAIN, TX_NOMINAL_MAX_GAIN, 1};
constexpr double RX_NOMINAL_MIN_GAIN = -16;
constexpr double RX_NOMINAL_MAX_GAIN = 15;
uhd::gain_range_t RX_NOMINAL_GAIN_RANGE{RX_NOMINAL_MIN_GAIN, RX_NOMINAL_MAX_GAIN, 1};

} // namespace

namespace scm {

struct phy::control
{
    control_type type;
    size_t chan;
    uhd::direction_t trx;
    uint8_t element;
    uint8_t value;
};

phy::phy(uhd::rfnoc::radio_control::sptr radio) : _radio(radio), _fw_version(0), _hw_version(0)
{
    const size_t shift = get_gpio_shift();

    const auto slot = radio->get_slot_name();
    UHD_ASSERT_THROW(slot == "A" || slot == "B");
    const auto gpio_bank  = (slot == "A") ? "GPIO0" : "GPIO1";
    const auto ch0_source = (slot == "A") ? "DB0_RF0" : "DB1_RF0";
    const auto ch1_source = (slot == "A") ? "DB0_RF1" : "DB1_RF1";
    const auto spi_source = (slot == "A") ? "DB0_SPI" : "DB1_SPI";

    // Set all GPIO pins to be driven by either SPI or CH0, except CH1 ATR
    std::vector<std::string> sources(12);
    sources[GPIO_PCLK]       = spi_source;
    sources[GPIO_RESERVED_1] = spi_source;
    sources[GPIO_SDI]        = spi_source;
    sources[GPIO_SDO]        = spi_source;
    sources[GPIO_CONTROL_CS] = spi_source;
    sources[GPIO_GENERAL_CS] = spi_source;
    sources[GPIO_ATR0_CH0]   = ch0_source;
    sources[GPIO_ATR1_CH1]   = ch1_source;
    sources[GPIO_LATCH_CH0]  = ch0_source;
    sources[GPIO_LATCH_CH1]  = ch0_source;
    sources[GPIO_RESERVED_2] = ch0_source;
    sources[GPIO_SPI_RESET]  = ch0_source;
    radio->get_mb_controller()->set_gpio_src(gpio_bank, sources);

    // Setup the ATR lines - only the two ATR lines are driven by ATR
    radio->set_gpio_attr("GPIO", "CTRL", (GPIO_ATR0_MASK | GPIO_ATR1_MASK) << shift);
    radio->set_gpio_attr("GPIO", "ATR_0X", 0);
    radio->set_gpio_attr("GPIO", "ATR_RX", 0);
    radio->set_gpio_attr("GPIO", "ATR_TX", (GPIO_ATR0_MASK | GPIO_ATR1_MASK) << shift);
    radio->set_gpio_attr("GPIO", "ATR_XX", (GPIO_ATR0_MASK | GPIO_ATR1_MASK) << shift);

    // Set the data direction register
    const uint32_t previous_ddr = radio->get_gpio_attr("GPIO", "DDR");
    const uint32_t new_ddr      = (previous_ddr & ~(0xFFF << shift)) | (GPIO_OUTPUT_MASK << shift);
    radio->set_gpio_attr("GPIO", "DDR", new_ddr);

    // Set the peripheral configurations, index = peripheral number
    UHD_ASSERT_THROW(radio->has_feature<uhd::features::spi_getter_iface>());
    std::vector<uhd::features::spi_periph_config_t> periph_configs(2);
    periph_configs[GENERAL_SPI] = {
        static_cast<uint8_t>(GPIO_GENERAL_CS + shift),
        static_cast<uint8_t>(GPIO_SDI + shift),
        static_cast<uint8_t>(GPIO_SDO + shift),
        static_cast<uint8_t>(GPIO_PCLK + shift),
    };
    periph_configs[CONTROL_SPI] = {
        static_cast<uint8_t>(GPIO_CONTROL_CS + shift),
        static_cast<uint8_t>(GPIO_SDI + shift),
        static_cast<uint8_t>(GPIO_SDO + shift),
        static_cast<uint8_t>(GPIO_PCLK + shift),
    };
    _spi = radio->get_feature<uhd::features::spi_getter_iface>().get_spi_ref(periph_configs);
    _dividers[GENERAL_SPI] = clock_to_divider(radio->get_rate(), 8.192e6);
    _dividers[CONTROL_SPI] = clock_to_divider(radio->get_rate(), 10.24e6);

    // Clear GPIO states and Reset SPI
    radio->set_gpio_attr("GPIO", "OUT", 0);

    // Excercise the SPI Interfaces to set the CS properly
    write_control({control_type::FILTER, 0, uhd::RX_DIRECTION, 0, 0});
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    read_general(SLOT_DETECT_REG);

    // Deassert SPI Reset
    radio->set_gpio_attr("GPIO", "OUT", GPIO_SPI_RESET_MASK << shift);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    // Validate SPI Communication
    validate_spi();

    // Get the Hardware and Firmware versions
    _hw_version = {static_cast<uint16_t>(read_general(HW_VER_REG) & 0xFFFF)};
    _fw_version = {static_cast<uint16_t>(read_general(FW_VER_REG) & 0xFFFF)};
    UHD_LOG_DEBUG(scm::NAME, "HW Version: " << _hw_version.to_string());
    UHD_LOG_DEBUG(scm::NAME, "FW Version: " << _fw_version.to_string());

    // Initialize Channels
    for (size_t chan = 0; chan < scm::CHANNELS_PER_SLOT; chan++) {
        // Set the Attenuator Latches GPIO State
        set_attn_latch(chan, true);
        // Initialize the Atten Latch and ATR Configuration
        set_triggers({chan, uhd::RX_DIRECTION}, SWTRIG_SRC_GPIO_VAL);
        set_triggers({chan, uhd::TX_DIRECTION}, SWTRIG_SRC_GPIO_VAL);
        // Initialize Gain Ranges (wait for coefficents to update)
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        update_gain_range({chan, uhd::RX_DIRECTION});
        update_gain_range({chan, uhd::TX_DIRECTION});
        // Enable the TX PA
        set_tx_pa(chan, true);
    }
    UHD_LOG_DEBUG(scm::NAME, "Initialized Slot " << slot);
}

void phy::validate_spi(void)
{
    UHD_LOG_DEBUG(
        scm::NAME, "Read Slot Detect 0x" << std::hex << (read_general(SLOT_DETECT_REG) & 0xFFFF));

    const uint32_t expected_pid = 0x901;
    const uint32_t pid          = read_general(PRODUCT_ID_REG) & 0xFFFF;
    UHD_LOG_DEBUG(scm::NAME, "Read PID 0x" << std::hex << pid << std::dec);
    if (pid != expected_pid) {
        std::stringstream msg;
        msg << "[" << scm::NAME << "] SPI communication failed!"
            << " Read PID: 0x" << std::hex << pid << std::dec << " Expected: 0x" << std::hex
            << expected_pid << std::dec;
        throw uhd::runtime_error(msg.str());
    }

    const uint32_t scratch = rand() % 0xFFFF;
    write_general(SCRATCH_REG, scratch);
    UHD_LOG_DEBUG(scm::NAME, "Wrote scratch: 0x" << std::hex << scratch);
    const uint32_t read_scratch = read_general(SCRATCH_REG) & 0xFFFF;
    UHD_LOG_DEBUG(scm::NAME, "Read scratch: 0x" << std::hex << read_scratch);
    if (read_scratch != scratch) {
        std::stringstream msg;
        msg << "[" << scm::NAME << "] SPI communication failed!";
        throw uhd::runtime_error(msg.str());
    }
}

uint32_t phy::read_general(const uint8_t address)
{
    uhd::spi_config_t config(uhd::spi_config_t::EDGE_RISE, _dividers[GENERAL_SPI]);
    const uint32_t data = static_cast<uint32_t>(address) << 24;
    return _spi->transact_spi(GENERAL_SPI, config, data, 32, true);
}

uint32_t phy::write_general(const uint8_t address, const uint16_t value)
{
    uhd::spi_config_t config(uhd::spi_config_t::EDGE_RISE, _dividers[GENERAL_SPI]);
    const uint32_t data = (1 << 31) | static_cast<uint32_t>(address) << 24
                          | static_cast<uint32_t>(value);
    return _spi->transact_spi(GENERAL_SPI, config, data, 32, true);
}

void phy::write_control(const control& control)
{
    UHD_ASSERT_THROW(control.chan < scm::CHANNELS_PER_SLOT);
    UHD_ASSERT_THROW(control.trx != uhd::DX_DIRECTION);
    UHD_ASSERT_THROW(control.element < 16);

    const uhd::spi_config_t config(uhd::spi_config_t::EDGE_RISE, _dividers[CONTROL_SPI]);
    const uint8_t ctrl_id = (static_cast<uint8_t>(control.type) << 6) | (control.chan << 5)
                            | ((control.trx == uhd::TX_DIRECTION ? 1 : 0) << 4) | (control.element);
    const uint32_t data = (static_cast<uint32_t>(ctrl_id) << 8)
                          | (static_cast<uint32_t>(control.value));
    _spi->transact_spi(CONTROL_SPI, config, data, 16, true);
}

void phy::set_attn_latch(const size_t chan, const bool enabled)
{
    const auto slot     = _radio->get_slot_name();
    const uint32_t mask = (chan == 0 ? GPIO_LATCH_CH0_MASK : GPIO_LATCH_CH1_MASK)
                          << get_gpio_shift();
    const uint32_t value     = enabled ? mask : 0;
    const uint32_t current   = _radio->get_gpio_attr("GPIO", "OUT");
    const uint32_t new_value = (current & ~mask) | (value & mask);
    _radio->set_gpio_attr("GPIO", "OUT", new_value);
}

bool phy::get_attn_latch(const size_t chan)
{
    const auto slot     = _radio->get_slot_name();
    const uint32_t mask = (chan == 0 ? GPIO_LATCH_CH0_MASK : GPIO_LATCH_CH1_MASK)
                          << get_gpio_shift();
    const uint32_t current = _radio->get_gpio_attr("GPIO", "OUT");
    return current & mask;
}

void phy::set_filter_spec(const path& path, const filter_spec& filter)
{
    write_control({
        control_type::FILTER,
        path.chan,
        path.trx,
        (path.trx == uhd::TX_DIRECTION) ? FILTER_TX_BAND_CTRL_ID : FILTER_RX_BAND_CTRL_ID,
        static_cast<uint8_t>((filter.band_id << 4) | filter.frequency_idx),
    });
    // It takes up to 500us to change filters and calibration coefficents
    std::this_thread::sleep_for(std::chrono::microseconds(500));
    update_gain_range(path);
}

void phy::set_gain(const path& path, const int8_t gain)
{
    write_control({
        control_type::GAIN,
        path.chan,
        path.trx,
        (path.trx == uhd::TX_DIRECTION) ? GAIN_TX_GAIN_CTRL_ID : GAIN_RX_GAIN_CTRL_ID,
        static_cast<uint8_t>(gain),
    });
}

void phy::set_tx_pa(const size_t chan, const bool enable)
{
    write_control({
        control_type::GAIN,
        chan,
        uhd::TX_DIRECTION,
        GAIN_TX_PA_CTRL_ID,
        static_cast<uint8_t>(enable),
    });
}

void phy::set_triggers(const path& path, const uint8_t value)
{
    write_control({
        control_type::SWTRIG,
        path.chan,
        path.trx,
        SWTRIG_LATCH_SRC_CTRL_ID,
        value,
    });
    write_control({
        control_type::SWTRIG,
        path.chan,
        path.trx,
        SWTRIG_DIR_SRC_CTRL_ID,
        value,
    });
}

void phy::update_gain_range(const path& path)
{
    const auto slot = _radio->get_slot_name();

    uint8_t address;
    uhd::gain_range_t* entry;

    // This was introduced with firmware 1.10
    if (_fw_version < version(1, 10))
        return;

    if (path.trx == uhd::RX_DIRECTION) {
        address = (path.chan == 0) ? GAIN_RANGE_RX_CH0_REG : GAIN_RANGE_RX_CH1_REG;
        entry   = &_rx_gain_range[std::make_pair(slot, path.chan)];
    } else {
        address = (path.chan == 0) ? GAIN_RANGE_TX_CH0_REG : GAIN_RANGE_TX_CH1_REG;
        entry   = &_tx_gain_range[std::make_pair(slot, path.chan)];
    }
    const auto value = static_cast<uint16_t>(read_general(address) & 0xFFFF);
    const double min = static_cast<int8_t>(value & 0xFF);
    const double max = static_cast<int8_t>((value >> 8) & 0xFF);

    if (min > max) {
        std::stringstream msg;
        msg << "[" << scm::NAME << "] SPI communication failed!";
        throw uhd::runtime_error(msg.str());
    }
    *entry = {min, max, 1};
}

uhd::gain_range_t phy::get_tx_gain_range(const size_t chan) const
{
    const auto slot = _radio->get_slot_name();
    const auto key  = std::make_pair(slot, chan);

    if (_tx_gain_range.find(key) == _tx_gain_range.end())
        return TX_NOMINAL_GAIN_RANGE;
    return _tx_gain_range.at(key);
}

uhd::gain_range_t phy::get_rx_gain_range(const size_t chan) const
{
    const auto slot = _radio->get_slot_name();
    const auto key  = std::make_pair(slot, chan);

    if (_rx_gain_range.find(key) == _rx_gain_range.end())
        return RX_NOMINAL_GAIN_RANGE;
    return _rx_gain_range.at(key);
}

size_t phy::clock_to_divider(const double radio_clk, double spi_clk) const
{
    spi_clk = (spi_clk < 1e6) ? 1e6 : spi_clk;
    spi_clk = (spi_clk > 40e6) ? 40e6 : spi_clk;
    return ((radio_clk / 2.0) / spi_clk) - 1.0;
}

uint32_t phy::get_gpio_shift(void) const
{
    const auto slot = _radio->get_slot_name();
    return (slot == "A") ? 0 : 12;
}
} // namespace scm
