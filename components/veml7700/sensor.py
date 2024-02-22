import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_GAIN,
    CONF_INTEGRATION_TIME,
    DEVICE_CLASS_ILLUMINANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
)

DEPENDENCIES = ["i2c"]
CONF_PSM = "psm"


CONF_AUTO_GAIN = "auto_gain"
CONF_AUTO_GAIN_THRESHOLD_HIGH = "auto_gain_threshold_high"
CONF_AUTO_GAIN_THRESHOLD_LOW = "auto_gain_threshold_low"
veml7700_ns = cg.esphome_ns.namespace("veml7700")

VEML7700Sensor = veml7700_ns.class_(
    "VEML7700Sensor", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)


VEML7700IntegrationTime = veml7700_ns.enum("VEML7700IntegrationTime")
VEML7700_INTEGRATION_TIMES = {
    "25ms": VEML7700IntegrationTime.VEML7700_INTEGRATION_25MS,
    "50ms": VEML7700IntegrationTime.VEML7700_INTEGRATION_50MS,
    "100ms": VEML7700IntegrationTime.VEML7700_INTEGRATION_100MS,
    "200ms": VEML7700IntegrationTime.VEML7700_INTEGRATION_200MS,
    "400ms": VEML7700IntegrationTime.VEML7700_INTEGRATION_400MS,
    "800ms": VEML7700IntegrationTime.VEML7700_INTEGRATION_800MS,
}

VEML7700Gain = veml7700_ns.enum("VEML7700Gain")
GAINS = {
    "1X": VEML7700Gain.VEML7700_GAIN_1X,
    "1/4X": VEML7700Gain.VEML7700_GAIN_0P25X,
    "1/8X": VEML7700Gain.VEML7700_GAIN_0P125X,
    "2X": VEML7700Gain.VEML7700_GAIN_2X,
    "AUTO": VEML7700Gain.VEML7700_GAIN_AUTO,

}

VEML7700PSM = veml7700_ns.enum("VEML7700PSM")
VEML7700_PSMS = {
    "1": VEML7700PSM.VEML7700_PSM_1,
    "2": VEML7700PSM.VEML7700_PSM_2,
    "3": VEML7700PSM.VEML7700_PSM_3,
    "4": VEML7700PSM.VEML7700_PSM_4,
}

def validate_integration_time(value):
    value = cv.positive_time_period_milliseconds(value).total_milliseconds
    return cv.enum(VEML7700_INTEGRATION_TIMES, int=True)(value)




CONFIG_SCHEMA = (
    sensor.sensor_schema(
        VEML7700Sensor,
        unit_of_measurement=UNIT_LUX,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ILLUMINANCE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Optional(CONF_AUTO_GAIN, default=True): cv.boolean,
            cv.Optional(CONF_AUTO_GAIN_THRESHOLD_HIGH, default="90%"): cv.percentage,
            cv.Optional(CONF_AUTO_GAIN_THRESHOLD_LOW, default="20%"): cv.percentage,
            cv.Optional(CONF_GAIN, default="2X"): cv.enum(GAINS, upper=True),
            cv.Optional(CONF_INTEGRATION_TIME, default="100ms"): validate_integration_time,
            cv.Optional(CONF_PSM, default="1"): cv.enum(VEML7700_PSMS, upper=True),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x10))
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_auto_gain(config[CONF_AUTO_GAIN]))
    cg.add(var.set_auto_gain_threshold_high(config[CONF_AUTO_GAIN_THRESHOLD_HIGH]))
    cg.add(var.set_auto_gain_threshold_low(config[CONF_AUTO_GAIN_THRESHOLD_LOW]))
    cg.add(var.set_gain(GAINS[config[CONF_GAIN]]))
    cg.add(var.set_integration_time(config[CONF_INTEGRATION_TIME]))
    cg.add(var.set_psm(config[CONF_PSM]))
