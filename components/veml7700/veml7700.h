#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace veml7700 {

/** Enum listing all conversion/integration time settings for the VEML7700
 *
 * Higher values mean more accurate results, but will take more energy/more time.
 */
enum VEML7700IntegrationTime {
  VEML7700_INTEGRATION_25MS  = 0x300,
  VEML7700_INTEGRATION_50MS  = 0x200,
  VEML7700_INTEGRATION_100MS = 0x00,
  VEML7700_INTEGRATION_200MS = 0x40,
  VEML7700_INTEGRATION_400MS = 0x80,
  VEML7700_INTEGRATION_800MS = 0xC0,
};

/** Enum listing all gain settings for the VEML7700.
 *
 * Higher values are better for low light situations, but can increase noise.
 */
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
  VEML7700_PSM_2 = 0x10,
  VEML7700_PSM_3 = 0x20,
  VEML7700_PSM_4 = 0x30,
};

/// This class includes support for the VEML7700 i2c ambient light sensor.
class VEML7700Sensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
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
  void set_integration_time(VEML7700IntegrationTime integration_time);

  /** Set the internal gain of the sensor. Can be useful for low-light conditions
   *
   * Possible values are:
   *
   *  - `sensor::VEML7700_ALS_GAIN_2X` (default)
   *  - `sensor::VEML7700_ALS_GAIN_1X`
   *  - `sensor::VEML7700_ALS_GAIN_0p25X`
   *  - `sensor::VEML7700_ALS_GAIN_0p125X`
   *
   * @param gain The new gain.
   */
  void set_gain(VEML7700Gain gain);

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
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

  bool veml7700_read_uint(uint8_t a_register, uint16_t *value);
  bool veml7700_write_uint(uint8_t a_register, uint8_t value);

 protected:
  float get_integration_time_ms_();
  void read_data_();
  float calculate_lx_(uint16_t als);

  VEML7700IntegrationTime integration_time_{VEML7700_INTEGRATION_100MS};
  VEML7700Gain gain_{VEML7700_ALS_GAIN_2X};
  VEML7700PSM psm_{VEML7700_ALS_GAIN_2X};
};

}  // namespace veml7700
}  // namespace esphome
