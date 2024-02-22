#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace veml7700 {
//static const char *const TAG = "VEML7700";

static const uint8_t ID_REG = 0x07;

// config
static const uint8_t CONFIGURATION_REGISTER    = 0x00;
static const uint16_t ALS_POWEROFF             = 0x1;
static const uint16_t ALS_POWERON              = 0x0;
static const uint16_t INTERRUPT_DISABLE        = 0x0;
static const uint16_t ALS_PERS_1               = 0x0;
static const uint16_t ALS_PERS_2               = 0x10;
static const uint16_t ALS_PERS_4               = 0x20;
static const uint16_t ALS_PERS_8               = 0x30;

//power save
const uint8_t POWER_SAVING_REGISTER = 0x03;
const uint8_t PSM_1                 = 0x0;
const uint8_t PSM_2                 = 0x02;
const uint8_t PSM_3                 = 0x04;
const uint8_t PSM_4                 = 0x06;
const uint8_t PSM_EN                = 0x01;
const uint8_t PSM_DIS               = 0x0;

// measurements
const uint8_t ALS_REGISTER          = 0x04;
const uint8_t WHITE_REGISTER        = 0x05;

// Other important constants
//
static const uint8_t DEVICE_ID = 0x81;

// Base multiplier value for lux computation
//
static const float LUX_MULTIPLIER_BASE = 0.0042;

// Enum for conversion/integration time settings for the VEML3235.
//
// Specific values of the enum constants are register values taken from the VEML3235 datasheet.
// Longer times mean more accurate results, but will take more energy/more time.
//
enum VEML7700IntegrationTime {
  VEML7700_INTEGRATION_25MS  = 0x300,
  VEML7700_INTEGRATION_50MS  = 0x200,
  VEML7700_INTEGRATION_100MS = 0x00,
  VEML7700_INTEGRATION_200MS = 0x40,
  VEML7700_INTEGRATION_400MS = 0x80,
  VEML7700_INTEGRATION_800MS = 0xC0,
};

// Enum for gain settings for the VEML3235.
// Higher values are better for low light situations, but can increase noise.
//
enum VEML7700Gain {
  VEML7700_GAIN_0p25X = 0x1800,
  VEML7700_GAIN_0p125X = 0x1000,
  VEML7700_GAIN_2X = 0x800,
  VEML7700_GAIN_1X = 0x0,
};
  
/** Enum listing all PSM settings for the VEML7700.
 *
 * Higher values are increasing the measurement frequency, but increases current cunsumption.
 */
enum VEML7700PSM {
  VEML7700_PSM_1 = 0x0,
  VEML7700_PSM_2 = 0x02,
  VEML7700_PSM_3 = 0x04,
  VEML7700_PSM_4 = 0x06,
};

/// This class includes support for the VEML7700 i2c ambient light sensor.
class VEML7700Sensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override { this->publish_state(this->read_lx_()); }
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Used by ESPHome framework. Does NOT actually set the value on the device.
  void set_auto_gain(bool auto_gain) { this->auto_gain_ = auto_gain; }
  void set_auto_gain_threshold_high(float auto_gain_threshold_high) {
    this->auto_gain_threshold_high_ = auto_gain_threshold_high;
  }
  void set_auto_gain_threshold_low(float auto_gain_threshold_low) {
    this->auto_gain_threshold_low_ = auto_gain_threshold_low;
  }

  void set_power_on(bool power_on) { this->power_on_ = power_on; }

  /** Set the internal gain of the sensor. Can be useful for low-light conditions
   *
   * Possible values are:
   *
   *  - `sensor::VEML7700_GAIN_2X` (default)
   *  - `sensor::VEML7700_GAIN_1X`
   *  - `sensor::VEML7700_GAIN_0p25X`
   *  - `sensor::VEML7700_GAIN_0p125X`
   *
   * @param gain The new gain.
   */
  void set_gain(VEML7700ComponentGain gain) { this->gain_ = gain; }

  /** Set the time that sensor values should be accumulated for.
   *
   * Longer means more accurate, but also mean more power consumption.
   *
   * Possible values are:
   *
   *  - `sensor::VEML7700_INTEGRATION_25MS`
   *  - `sensor::VEML7700_INTEGRATION_50MS`
   *  - `sensor::VEML7700_INTEGRATION_100MS` (default)
   *  - `sensor::VEML7700_INTEGRATION_200MS`
   *  - `sensor::VEML7700_INTEGRATION_400MS`  
   *  - `sensor::VEML7700_INTEGRATION_800MS`
   *
   * @param integration_time The new integration time.
   */
  void set_integration_time(VEML7700ComponentIntegrationTime integration_time) {
    this->integration_time_ = integration_time;
  }

  bool auto_gain() { return this->auto_gain_; }
  float auto_gain_threshold_high() { return this->auto_gain_threshold_high_; }
  float auto_gain_threshold_low() { return this->auto_gain_threshold_low_; }
  VEML7700ComponentGain gain() { return this->gain_; }
  VEML7700ComponentIntegrationTime integration_time() { return this->integration_time_; }

  // Updates the configuration register on the device
  bool refresh_config_reg(bool force_on = false);

  /** Set the PowesSavingMode of the sensor. Sets the measurement period
   *
   * Possible values are:
   *
   *  - `sensor::VEML7700_PSM_1` (default)
   *  - `sensor::VEML7700_PSM_2`
   *  - `sensor::VEML7700_PSM_3`
   *  - `sensor::VEML7700_PSM_4`
   *
   * @param psm The new PSM.
   */
  void set_psm(VEML7700PSM psm);  

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)

  float get_setup_priority() const override;

  bool veml7700_read_uint(uint8_t a_register, uint16_t *value);
  bool veml7700_write_uint(uint8_t a_register, uint16_t value);
  bool veml7700_read_bytes_16(uint8_t a_register, uint16_t *value) ;

 protected:
  float read_lx_();
  void adjust_gain_(uint16_t als_raw_value);

  bool auto_gain_{true};
  bool power_on_{true};
  float auto_gain_threshold_high_{0.9};
  float auto_gain_threshold_low_{0.2};

  VEML7700Gain gain_{VEML7700_GAIN_2X};
  VEML7700IntegrationTime integration_time_{VEML7700_INTEGRATION_100MS};
  VEML7700PSM psm_{VEML7700_PSM_4};
};

}  // namespace veml7700
}  // namespace esphome
