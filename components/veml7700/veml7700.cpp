#include "veml7700.h"
#include "esphome/core/log.h"

namespace esphome {
namespace veml7700 {

static const char *const TAG = "VEML7700";

static const uint8_t TSL2561_COMMAND_BIT = 0x80;
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

void VEML7700Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up VEML7700...");

  uint16_t integration_time = this->integration_time_;
  uint16_t gain = this->gain_;
  uint16_t psm = this->psm_;
    
  if (!this->veml7700_write_uint(CONFIGURATION_REGISTER, ALS_POWERON | integration_time | gain)) 
  {
     this->mark_failed();
     return;   
  }
  else
  {
    ESP_LOGCONFIG(TAG, "Power on, integration time, and gain are set %u", ALS_POWERON | integration_time | gain);
  }
  
  if (!this->veml7700_write_uint(POWER_SAVING_REGISTER, PSM_EN | psm))
  {
     this->mark_failed();
     return;   
  }
  else
  {
    ESP_LOGCONFIG(TAG, "Power save registers are set %u", PSM_EN | psm);
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
    case VEML7700_GAIN_0p25X :
      a_gain = 0.25;
    case VEML7700_GAIN_0p125X :
      a_gain = 0.125;
    default:
      a_gain = 2;
  }
  ESP_LOGCONFIG(TAG, "  Gain: %.3f x", a_gain);
  ESP_LOGCONFIG(TAG, "  Integration Time: %.1f ms", this->get_integration_time_ms_());

  LOG_UPDATE_INTERVAL(this);
}
void VEML7700Sensor::update() {
  this->read_data_();
}

float VEML7700Sensor::calculate_lx_(uint16_t als) {
  float als_f = als;
  float magic_number = 0.0288;
  switch (this->integration_time_)
  {
    case VEML7700_INTEGRATION_200MS:
      magic_number = 0.0144;
    case VEML7700_INTEGRATION_400MS:
      magic_number = 0.0072;
    case VEML7700_INTEGRATION_800MS:
      magic_number = 0.0036;
    case VEML7700_INTEGRATION_50MS:
      magic_number = 0.0576;
    case VEML7700_INTEGRATION_25MS:
      magic_number = 0.1152;
    default:
      magic_number = 0.0288;
  } 
  float a_gain = 1;
  switch (this->gain_)
  {
    case VEML7700_GAIN_1X :
      a_gain = 0.5;
    case VEML7700_GAIN_0p25X :
      a_gain = 8;
    case VEML7700_GAIN_0p125X :
      a_gain = 16;
    default:
      a_gain = 1;
  }
  float lx = als_f * magic_number * a_gain;
  return lx;
}
  
void VEML7700Sensor::read_data_() {
  uint16_t als;
  if (!this->veml7700_read_uint(ALS_REGISTER, &als)) {
    this->status_set_warning();
    return;
  }
  ESP_LOGD(TAG, "Got illuminance=%u ", als);
  float lx = this->calculate_lx_(als);
  ESP_LOGD(TAG, "Got illuminance=%.1flx", lx);
  this->publish_state(lx);
  this->status_clear_warning();
}
  
float VEML7700Sensor::get_integration_time_ms_() {
  switch (this->integration_time_) {
    case VEML7700_INTEGRATION_200MS:
      return 200.0f;
    case VEML7700_INTEGRATION_400MS:
      return 400.0f;
    case VEML7700_INTEGRATION_800MS:
      return 800.0f;
    case VEML7700_INTEGRATION_50MS:
      return 50.0f;
    case VEML7700_INTEGRATION_25MS:
      return 25.0f;
    default:
      return 100.0f;
  }
  return 0.0f;
}
  
float VEML7700Sensor::get_gain_() {
  switch (this->gain_) {
    case VEML7700_GAIN_1X :
      return 1.0f;
    case VEML7700_GAIN_0p25X :
      return 0.25f;
    case VEML7700_GAIN_0p125X :
      return 0.125f;
    default:
      return 2.0f;
  }
  return 0.0f;
}
  
void VEML7700Sensor::set_integration_time(VEML7700IntegrationTime integration_time) {
  this->integration_time_ = integration_time;
}
void VEML7700Sensor::set_gain(VEML7700Gain gain) { this->gain_ = gain; }
void VEML7700Sensor::set_psm(VEML7700PSM psm) { this->psm_ = psm; }
  
float VEML7700Sensor::get_setup_priority() const { return setup_priority::DATA; }

bool VEML7700Sensor::veml7700_read_uint(uint8_t a_register, uint16_t *value) {
  uint8_t data[2];
  if (!this->read_bytes(a_register, data, 2))
    return false;
  const uint16_t hi = data[1];
  const uint16_t lo = data[0];
  *value = (hi << 8) | lo;
  return true;
}

bool VEML7700Sensor::veml7700_write_uint(uint8_t a_register, uint16_t value) {
  return this->write_byte_16(a_register, value);
}
  
}  // namespace tsl2561
}  // namespace esphome
