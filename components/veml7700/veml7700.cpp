#include "veml7700.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace veml7700 {

static const char *const TAG = "veml7700.sensor";

void VEML7700Sensor::setup() {
  uint8_t device_id[] = {0, 0};

  ESP_LOGCONFIG(TAG, "Setting up VEML7700 '%s'...", this->name_.c_str());
  if (!this->refresh_config_reg()) {
    ESP_LOGE(TAG, "Unable to write configuration");
    this->mark_failed();
    return;
  }

  uint16_t data = PSM_EN;
  data |= (uint16_t(this->psm_));
  ESP_LOGD(TAG, "Enabling PSM: Writing 0x%.4x to register 0x%.2x", data, POWER_SAVING_REGISTER);
  if (!this->write_byte_16(POWER_SAVING_REGISTER, data)) {
    ESP_LOGE(TAG, "Unable to write PSM register");
    this->mark_failed();
    return;
  }
  
  if ((this->write(&ID_REG, 1, false) != i2c::ERROR_OK) || !this->read_bytes_raw(device_id, 2)) {
    ESP_LOGE(TAG, "Unable to read ID");
    this->mark_failed();
    return;
  } else if (device_id[0] != DEVICE_ID) {
    ESP_LOGE(TAG, "Incorrect device ID - expected 0x%.2x, read 0x%.2x", DEVICE_ID, device_id[0]);
    this->mark_failed();
    return;
  }
}

bool VEML7700Sensor::refresh_config_reg(bool force_on) 
{
  
  uint16_t data = this->power_on_ || force_on ? ALS_POWERON : ALS_POWEROFF;
  data |= (uint16_t(this->integration_time_));
  data |= (uint16_t(this->gain_));

  //swap upper and lower bytes
  uint8_t array[2];
  array[0]=data & 0xff;
  array[1]=(data >> 8);
  data = encode_uint16(array[0], array[1]);
  
  ESP_LOGD(TAG, "Turn on ALS and set up configuration: Writing 0x%.4x to register 0x%.2x", data, CONFIGURATION_REGISTER);
  return this->write_byte_16(CONFIGURATION_REGISTER, data);

}

float VEML7700Sensor::read_lx_() {
  if (!this->power_on_) {  // if off, turn on
    if (!this->refresh_config_reg(true)) {
      ESP_LOGW(TAG, "Turning on failed");
      this->status_set_warning();
      return NAN;
    }
    delay(4);  // from RM: a wait time of 4 ms should be observed before the first measurement is picked up, to allow
               // for a correct start of the signal processor and oscillator
  }

  //Only for debug
  //uint16_t data = this->power_on_ ? ALS_POWERON : ALS_POWEROFF;
  //data |= (uint16_t(this->integration_time_));
  //data |= (uint16_t(this->gain_));
  //ESP_LOGD(TAG, "Expected configuration raw = 0x%.4x", data);
  //uint8_t conf_regs[] = {0, 0};
  //if ((this->write(&CONFIGURATION_REGISTER, 1, false) != i2c::ERROR_OK) || !this->read_bytes_raw(conf_regs, 2)) {
  //  ESP_LOGD(TAG, "'Unable to read configuration register");
  //}
  //uint16_t config_value = encode_uint16(conf_regs[1], conf_regs[0]);
  //ESP_LOGD(TAG, "Read configuration raw = 0x%.4x", config_value);
  
  uint8_t als_regs[] = {0, 0};
  if ((this->write(&ALS_REGISTER, 1, false) != i2c::ERROR_OK) || !this->read_bytes_raw(als_regs, 2)) {
    this->status_set_warning();
    return NAN;
  }

  this->status_clear_warning();

  float als_raw_value_multiplier = LUX_MULTIPLIER_BASE;
  uint16_t als_raw_value = encode_uint16(als_regs[1], als_regs[0]);
  // determine multiplier value based on gains and integration time
  switch (this->gain_) {
    case VEML7700_GAIN_0p125X:
      als_raw_value_multiplier *= 16;
      break;
    case VEML7700_GAIN_0p25X:
      als_raw_value_multiplier *= 8;
      break;
    case VEML7700_GAIN_1X:
      als_raw_value_multiplier *= 2;
      break;
    case VEML7700_GAIN_2X:
      als_raw_value_multiplier *=1;
      break;
    default:
      break;
  }
  
  switch (this->integration_time_)
  {
    case VEML7700_INTEGRATION_25MS:
      als_raw_value_multiplier *= 32;
      break;    
    case VEML7700_INTEGRATION_50MS:
      als_raw_value_multiplier *= 16;
      break;
    case VEML7700_INTEGRATION_100MS:
      als_raw_value_multiplier *= 8;
      break;
    case VEML7700_INTEGRATION_200MS:
      als_raw_value_multiplier *= 4;
      break;
    case VEML7700_INTEGRATION_400MS:
      als_raw_value_multiplier *= 2;
      break;
    case VEML7700_INTEGRATION_800MS:
      als_raw_value_multiplier *= 1;
      break;
    default:
      break;
  } 
  // finally, determine and return the actual lux value
  float lx = float(als_raw_value) * als_raw_value_multiplier;
  ESP_LOGD(TAG, "'%s': ALS raw = %u, multiplier = %.5f", this->get_name().c_str(), als_raw_value,
            als_raw_value_multiplier);
  ESP_LOGD(TAG, "'%s': Illuminance = %.4flx", this->get_name().c_str(), lx);

  if (!this->power_on_) {  // turn off if required
    if (!this->refresh_config_reg()) {
      ESP_LOGW(TAG, "Turning off failed");
      this->status_set_warning();
    }
  }

  if (this->auto_gain_) {
    this->adjust_gain_(als_raw_value);
  }

  return lx;
}

void VEML7700Sensor::adjust_gain_(const uint16_t als_raw_value) {
  // if the raw value is not too high and not too low, nothing to do, exit
  if ((als_raw_value > UINT16_MAX * this->auto_gain_threshold_low_) &&
      (als_raw_value < UINT16_MAX * this->auto_gain_threshold_high_)) 
  {
    ESP_LOGD(TAG, "No change needed for ALS raw = %u, low_lim: %u, high_lim: %u" , 
      als_raw_value,(unsigned int)(UINT16_MAX * this->auto_gain_threshold_low_),
      (unsigned int)(UINT16_MAX * this->auto_gain_threshold_high_));
    return;
  }

  // if the raw value is too high reset everything and exit
  if (als_raw_value >= UINT16_MAX * this->auto_gain_threshold_high_) {  // over-saturated, reset all gains and start over
    this->gain_ = VEML7700_GAIN_0p125X;
    this->integration_time_ = VEML7700_INTEGRATION_25MS;
    this->refresh_config_reg();
    ESP_LOGD(TAG, "Raw value is too high, reset to minimum for  ALS raw = %u", als_raw_value);
    return;
  }
  //if the raw value is too low, than first set the gain only
  if (this->gain_ != VEML7700_GAIN_2X) {  // increase gain if possible
    switch (this->gain_) {
      case VEML7700_GAIN_0p125X:
        this->gain_ = VEML7700_GAIN_0p25X;
         ESP_LOGD(TAG, "Gain increased for ALS raw = %u to 1/4" , als_raw_value);
        break;
      case VEML7700_GAIN_0p25X:
        this->gain_ = VEML7700_GAIN_1X;
        ESP_LOGD(TAG, "Gain increased for ALS raw = %u to 1" , als_raw_value);
        break;
      case VEML7700_GAIN_1X:
        this->gain_ = VEML7700_GAIN_2X;
        ESP_LOGD(TAG, "Gain increased for ALS raw = %u to 2" , als_raw_value);
        break;
      default:
        break;
    }
    this->refresh_config_reg();
    return;
  }

  // digital gain is maxed out; reset it and try to increase integration time
  if (this->integration_time_ != VEML7700_INTEGRATION_800MS) {  // increase integration time if possible
    switch (this->integration_time_) {
      case VEML7700_INTEGRATION_25MS:
        this->integration_time_ = VEML7700_INTEGRATION_50MS;
        ESP_LOGD(TAG, "Integration time increased for ALS raw = %u to 50ms gain to 1/8" , als_raw_value);
        break;
      case VEML7700_INTEGRATION_50MS:
        this->integration_time_ = VEML7700_INTEGRATION_100MS;
        ESP_LOGD(TAG, "Integration time increased for ALS raw = %u to 100ms gain to 1/8" , als_raw_value);
        break;
      case VEML7700_INTEGRATION_100MS:
        this->integration_time_ = VEML7700_INTEGRATION_200MS;
        ESP_LOGD(TAG, "Integration time increased for ALS raw = %u to 200ms gain to 1/8" , als_raw_value);
        break;
      case VEML7700_INTEGRATION_200MS:
        this->integration_time_ = VEML7700_INTEGRATION_400MS;
        ESP_LOGD(TAG, "Integration time increased for ALS raw = %u to 400ms gain to 1/8" , als_raw_value);
        break;
      case VEML7700_INTEGRATION_400MS:
        this->integration_time_ = VEML7700_INTEGRATION_800MS;
        ESP_LOGD(TAG, "Integration time increased for ALS raw = %u to 800ms gain to 1/8" , als_raw_value);
        break;
      default:
        break;
    }
    this->gain_ = VEML7700_GAIN_0p125X;
    this->refresh_config_reg();
    return;
  }
}


void VEML7700Sensor::dump_config() {
  LOG_SENSOR("", "VEML7700", this);
  LOG_I2C_DEVICE(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with VEML7700 failed!");
  }

  float a_gain = 0;
  switch (this->gain_)
  {
    case VEML7700_GAIN_1X :
      a_gain = 1;
      break;
    case VEML7700_GAIN_0p25X :
      a_gain = 0.25;
      break;
    case VEML7700_GAIN_0p125X :
      a_gain = 0.125;
      break;
    case VEML7700_GAIN_2X :
      a_gain = 2;
      break;
    default:
      break;
  }

  float an_int_time = 0;
  switch (this->integration_time_)
  {
    case VEML7700_INTEGRATION_100MS:
      an_int_time = 100;
      break;
    case VEML7700_INTEGRATION_200MS:
      an_int_time = 200;
      break;
    case VEML7700_INTEGRATION_400MS:
      an_int_time = 400;
      break;
    case VEML7700_INTEGRATION_800MS:
      an_int_time = 800;
      break;
    case VEML7700_INTEGRATION_50MS:
      an_int_time = 50;
      break;
    case VEML7700_INTEGRATION_25MS:
      an_int_time = 25;
      break;
    default:
      break;
  }

  float a_psm = 0;
  float a_wakeuptime = 0;
  switch (this->psm_)
  {
    case PSM_1 :
      a_psm = 1;
      a_wakeuptime = 500;
      break;
    case PSM_2 :
      a_psm = 2;
      a_wakeuptime = 1000;
      break;
    case PSM_3 :
      a_psm = 3;
      a_wakeuptime = 2000;
      break;
    case PSM_4 :
      a_psm = 4;
      a_wakeuptime = 4000;
      break;
    default:
      break;
  }
  
  ESP_LOGCONFIG(TAG, "  Gain: %.0fX; 0x%.4x", a_gain, this->gain_);
  ESP_LOGCONFIG(TAG, "  Integration Time: %.0fms; 0x%.3x", an_int_time, this->integration_time_);
  ESP_LOGCONFIG(TAG, "  PSM: %.0f, wake-up-time: %.0fms; 0x%.2x",a_psm, a_wakeuptime, this->psm_);

  LOG_UPDATE_INTERVAL(this);
}

}  // namespace tsl2561
}  // namespace esphome
